import sys
import os
print("Start")
sys.path.insert(0, os.path.expanduser('~/.local/lib/python3.10/site-packages'))
print("Insert path")
from isaacsim import SimulationApp
print("Import SimulationApp")
kit = SimulationApp({"renderer": "RaytracedLighting", "headless": False})
# import omni.kit.commands
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.extensions import get_extension_path_from_name
from pxr import Gf, PhysxSchema, Sdf, UsdLux, UsdPhysics, UsdGeom
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import GeometryPrim
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_prefix
import omni.kit.commands

# Derive workspace src path: install/pcgrad -> ../../src
_pkg_prefix = get_package_prefix('pcgrad')
_ws_src = os.path.join(_pkg_prefix, '..', '..', 'src')
ROBOT_DESCRIPTION_DIR = os.path.realpath(os.path.join(_ws_src, 'robot_description'))



class VisionProcessingNode(Node):
    def __init__(self):
        super().__init__('vision_processing_node')
        
        # 1. Subscriber: Lắng nghe topic ảnh từ camera (Gazebo, Isaac Sim, hoặc cam thực tế)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # 2. Publisher: Gửi kết quả sau khi xử lý (ví dụ: tọa độ vật thể)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joints_state', 10)
        
        self.get_logger().info('Vision Node đã khởi động và đang chờ dữ liệu...')
        stage = omni.usd.get_context().get_stage()
        # 2. Đặt Default Prim là /World
        # Lưu ý: Prim này phải tồn tại trong stage trước khi đặt làm Default
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            stage.SetDefaultPrim(world_prim)
            print("Đã đặt /World làm Default Prim")
        else:
            # Nếu /World chưa tồn tại, bạn có thể tạo nó trước
            from pxr import UsdGeom
            UsdGeom.Xform.Define(stage, "/World")
            stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
            print("Đã tạo và đặt /World làm Default Prim")


        my_world = World(stage_units_in_meters=1.0)

        cube_2 = my_world.scene.add(
            DynamicCuboid(
                prim_path="/World/new_cube_2",
                name="cube_2",
                position=np.array([-9.0, 0.8, 96.12834]),
                scale=np.array([30.0,25.0, 0.4]),
                size=1.0,
                color=np.array([255, 0, 0]),
                mass=200
            )
        )

        # Setting up import configuration:
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = False
        import_config.distance_scale = 0.1

        # Get path to extension data:
        extension_path = get_extension_path_from_name("isaacsim.asset.importer.urdf")
        # Import URDF, prim_path contains the path the path to the usd prim in the stage.
        urdf_path = os.path.join(ROBOT_DESCRIPTION_DIR, 'ARM', 'robot.urdf')
        print(f'URDF path: {urdf_path}')
        status, robot_prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=urdf_path,
            import_config=import_config,
            get_articulation_root=True,
        )
        # Get stage handle
        stage = omni.usd.get_context().get_stage()

        # Enable physics
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
        # Set gravity
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        # Set solver settings
        PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
        physxSceneAPI.CreateEnableCCDAttr(True)
        physxSceneAPI.CreateEnableStabilizationAttr(True)
        physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
        physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
        physxSceneAPI.CreateSolverTypeAttr("TGS")

        # Add ground plane
        omni.kit.commands.execute(
            "AddGroundPlaneCommand",
            stage=stage,
            planePath="/groundPlane",
            axis="Z",
            size=1500.0,
            position=Gf.Vec3f(0, 0, -0.50),
            color=Gf.Vec3f(0.5),
        )

        # Add lighting
        distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)

        # Get handle to the Drive API for both wheels
        self.joint0_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/World/robot/joints/joint0"), "angular")
        self.joint1_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/World/robot/joints/joint1"), "angular")
        self.joint2_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/World/robot/joints/joint2"), "angular")
        self.joint3_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/World/robot/joints/joint3"), "angular")
        # self.jointleft_drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath("/World/robot/joints/left_finger"), "linear")
        # self.jointright_drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath("/World/robot/joints/right_finger"), "linear")
        # Set the drive damping, which controls the strength of the velocity drive
        self.joint0_drive.GetDampingAttr().Set(15000)
        self.joint1_drive.GetDampingAttr().Set(15000)
        self.joint2_drive.GetDampingAttr().Set(15000)
        self.joint3_drive.GetDampingAttr().Set(15000)
        # self.jointleft_drive.CreateDampingAttr().Set(15000)
        # self.jointright_drive.CreateDampingAttr().Set(15000)

        # Set the drive stiffness, which controls the strength of the position drive
        # In this case because we want to do velocity control this should be set to zero
        self.joint0_drive.GetStiffnessAttr().Set(10000)
        self.joint1_drive.GetStiffnessAttr().Set(10000)
        self.joint2_drive.GetStiffnessAttr().Set(10000)
        self.joint3_drive.GetStiffnessAttr().Set(10000)
        # self.jointleft_drive.CreateStiffnessAttr().Set(10000)
        # self.jointright_drive.CreateStiffnessAttr().Set(10000)

        asset_path = os.path.join(ROBOT_DESCRIPTION_DIR, 'table', 'source', 'TABLE.usd')
        simulation_context = SimulationContext()
        add_reference_to_stage(asset_path, "/World/table")

        tableXForm = XFormPrim(prim_paths_expr='/World/table', name="table")
        table_quat = euler_angles_to_quat(np.array([90, 0, 0]), degrees=True)
        tableXForm.set_world_poses(
            positions=np.array([[1.0, 1.0, 92.0]]), 
            orientations=table_quat.reshape(1, 4)
        )
        table = GeometryPrim(prim_paths_expr="/World/table", name="table_geo")
        # Bật collision (Nếu chưa được bật khi khởi tạo)
        table.enable_collision()
        # Quan trọng: Thiết lập kiểu xấp xỉ va chạm (Collision Approximation)
        # Các kiểu phổ biến: "convexHull", "convexDecomposition" (cho mesh phức tạp), "boundingCube", "none" (triangle mesh)
        table.set_collision_approximations(approximation_types=["convexHull"])
        UsdGeom.Xform.Define(stage, "/World/table/base")

        # Gắn RigidBodyAPI cho /World/table/base để physics engine nhận diện nó là body
        base_prim = stage.GetPrimAtPath("/World/table/base")
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(base_prim)
        rigid_body.CreateKinematicEnabledAttr(True)  # kinematic = không bị trọng lực kéo

        robotXForm = XFormPrim(prim_paths_expr='/World/robot', name="robot")
        robot_quat = euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)
        robotXForm.set_world_poses(
            positions=np.array([[0.0, 0.0, 97.4]]), 
            orientations=robot_quat.reshape(1, 4)
        )

        # Tạo FixedJoint trực tiếp bằng USD API 
        # Tạo FixedJoint trực tiếp bằng USD API 
        # Dùng body0 = mb1 (link gốc robot), body1 = new_cube_2
        fixed_joint = UsdPhysics.FixedJoint.Define(stage, "/World/robot/mb1/fixed_joint")
        fixed_joint.CreateBody0Rel().SetTargets([Sdf.Path("/World/robot/mb1")])
        fixed_joint.CreateBody1Rel().SetTargets([Sdf.Path("/World/new_cube_2")])

        # Để tránh cảnh báo "disjointed body transforms", ta cần set LocalPos/Rot
        # sao cho joint khớp với vị trí hiện tại của 2 body trong không gian thực.
        # Set LocalRot0 theo yêu cầu người dùng: (-90, 0, 180) Euler
        rot0_quat_np = euler_angles_to_quat(np.array([-90.0, 0.0, 180.0]), degrees=True) # [w, x, y, z]
        quat0 = Gf.Quatd(rot0_quat_np[0], rot0_quat_np[1], rot0_quat_np[2], rot0_quat_np[3])

        # Ta muốn T_world_0 * T_local_0 = T_world_1 * T_local_1
        # => T_local_1 = T_world_1^-1 * T_world_0 * T_local_0

        prim0 = stage.GetPrimAtPath("/World/robot/mb1")
        prim1 = stage.GetPrimAtPath("/World/new_cube_2")
        T_w_0 = omni.usd.utils.get_world_transform_matrix(prim0)
        T_w_1 = omni.usd.utils.get_world_transform_matrix(prim1)

        # Xây dựng T_local_0 (pos=0, rot=user_quat)
        T_l_0 = Gf.Matrix4d().SetRotate(quat0)
        T_l_0.SetTranslateOnly(Gf.Vec3d(0, 0, 0))

        # Tính T_local_1
        T_l_1 = T_l_0 * T_w_0 * T_w_1.GetInverse() # Lưu ý thứ tự nhân ma trận trong Gf (row-major)

        rel_pos1 = T_l_1.ExtractTranslation()
        rel_quat1 = T_l_1.ExtractRotation().GetQuaternion()

        # Áp dụng cho Joint
        fixed_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        fixed_joint.CreateLocalRot0Attr().Set(Gf.Quatf(quat0.GetReal(), Gf.Vec3f(quat0.GetImaginary())))

        fixed_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(rel_pos1))
        fixed_joint.CreateLocalRot1Attr().Set(Gf.Quatf(rel_quat1.GetReal(), Gf.Vec3f(rel_quat1.GetImaginary())))

        # Loại bỏ joint này khỏi articulation để tránh conflict
        fixed_joint.CreateExcludeFromArticulationAttr().Set(True)

        # Start simulation
        omni.timeline.get_timeline_interface().play()
        # perform one simulation step so physics is loaded and dynamic control works.
        kit.update()
        art = Articulation(robot_prim_path)
        art.initialize()

        if not art.is_physics_handle_valid():
            print(f"{robot_prim_path} is not an articulation")
        else:
            print(f"Got articulation ({robot_prim_path})")
        kit.update()
        print("Start")

    def image_callback(self, msg):
        data = msg.data
        
        # Gán lại các biến từ list để dễ xử lý
        j1 = data[0]
        j2 = data[1]
        j3 = data[2]    
        j4 = data[3]
        gripper = data[4]

        # In thông tin ra console
        self.get_logger().info(
            f'Nhận dữ liệu: Joint1={j1:.2f}, Joint2={j2:.2f}, Joint3={j3:.2f}, Joint4={j4:.2f}, Gripper={gripper:.2f}'
        )
        self.joint0_drive.GetTargetPositionAttr().Set(j1)
        self.joint1_drive.GetTargetPositionAttr().Set(j2)
        self.joint2_drive.GetTargetPositionAttr().Set(j3)
        self.joint3_drive.GetTargetPositionAttr().Set(j4)
        # self.jointleft_drive.GetTargetPositionAttr().Set(dem)
        # self.jointright_drive.GetTargetPositionAttr().Set(dem)
        
        kit.update()

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessingNode()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)  # Non-blocking ROS callback
            kit.update()  # Update Isaac Sim mỗi frame
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()