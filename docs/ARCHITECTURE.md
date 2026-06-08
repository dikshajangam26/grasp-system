# System Architecture

## Data Flow Diagram
This flowchart captures the hybrid architecture, showing how the Vision node splits its outputs between direct velocity commands and the machine learning pipeline.

```mermaid
graph TD
    %% Styling
    classDef hardware fill:#e1f5fe,stroke:#01579b,stroke-width:2px
    classDef software fill:#ffffff,stroke:#333333,stroke-width:2px,rx:5,ry:5

    %% Nodes - Note the double quotes around text with HTML!
    Cam(("Camera Input")):::hardware
    Vision["Vision-Based Servoing<br/><i>(Real-time tracking)</i>"]:::software
    Predictor["Grasp Quality Predictor<br/><i>(ML inference)</i>"]:::software
    IK["IK Solver & Planner<br/><i>(Motion planning)</i>"]:::software
    ArmCtrl["Arm Control Interface<br/><i>(Joint commands)</i>"]:::software
    Robot(("Robotic Arm")):::hardware

    %% Edges
    Cam --> Vision
    
    Vision -- "Velocity Cmd" --> ArmCtrl
    Vision -- "Marker Pose" --> Predictor
    
    Predictor -- "Quality Score" --> IK
    
    IK -- "Joint Trajectory" --> ArmCtrl
    
    ArmCtrl --> Robot
```
# Module Dependencies
## This mindmap visually represents how the Master Controller orchestrates the sub-modules and their respective libraries.
```mermaid
mindmap
  root(("grasp_controller.py<br/>(Orchestrator)"))
    vision_servoing
      opencv("opencv (ArUco detection)")
      numpy("numpy (math)")
    grasp_predictor
      ml("tensorflow/torch (ML inference)")
      sklearn("sklearn (preprocessing)")
    ik_planner
      ikpy("ikpy (kinematics)")
      numpy_ik("numpy (trajectory planning)")
    performance_metrics
      time("time (benchmarking)")
```

# ROS 2 Node Graph
## This represents the ROS 2 pub/sub network flowing horizontally.
```mermaid
graph LR
    %% Styling
    classDef rosnode fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px,rx:10,ry:10
    classDef topic fill:#fff3e0,stroke:#e65100,stroke-width:1px,stroke-dasharray: 5 5

    %% Nodes
    Cam[Camera Driver]:::rosnode
    Vision[VisionServo Node]:::rosnode
    Pred[GraspPredictor Node]:::rosnode
    IK[IKPlanner Node]:::rosnode
    Arm[ArmController Node]:::rosnode

    %% Topics (Edges)
    Cam -- "/camera/image" --> Vision
    Vision -- "/servo/velocity" --> Pred
    Pred -- "/grasp/quality_score" --> IK
    IK -- "/arm/trajectory_goal" --> Arm

```