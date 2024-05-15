;; Auto-generated. Do not edit!


(when (boundp 'bboat_pkg::next_target_serv)
  (if (not (find-package "BBOAT_PKG"))
    (make-package "BBOAT_PKG"))
  (shadow 'next_target_serv (find-package "BBOAT_PKG")))
(unless (find-package "BBOAT_PKG::NEXT_TARGET_SERV")
  (make-package "BBOAT_PKG::NEXT_TARGET_SERV"))
(unless (find-package "BBOAT_PKG::NEXT_TARGET_SERVREQUEST")
  (make-package "BBOAT_PKG::NEXT_TARGET_SERVREQUEST"))
(unless (find-package "BBOAT_PKG::NEXT_TARGET_SERVRESPONSE")
  (make-package "BBOAT_PKG::NEXT_TARGET_SERVRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass bboat_pkg::next_target_servRequest
  :super ros::object
  :slots (_req ))

(defmethod bboat_pkg::next_target_servRequest
  (:init
   (&key
    ((:req __req) nil)
    )
   (send-super :init)
   (setq _req __req)
   self)
  (:req
   (&optional (__req :null))
   (if (not (eq __req :null)) (setq _req __req)) _req)
  (:serialization-length
   ()
   (+
    ;; bool _req
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _req
       (if _req (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _req
     (setq _req (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass bboat_pkg::next_target_servResponse
  :super ros::object
  :slots (_next_trgt_pose _continuing_mission ))

(defmethod bboat_pkg::next_target_servResponse
  (:init
   (&key
    ((:next_trgt_pose __next_trgt_pose) (instance geometry_msgs::Pose :init))
    ((:continuing_mission __continuing_mission) nil)
    )
   (send-super :init)
   (setq _next_trgt_pose __next_trgt_pose)
   (setq _continuing_mission __continuing_mission)
   self)
  (:next_trgt_pose
   (&rest __next_trgt_pose)
   (if (keywordp (car __next_trgt_pose))
       (send* _next_trgt_pose __next_trgt_pose)
     (progn
       (if __next_trgt_pose (setq _next_trgt_pose (car __next_trgt_pose)))
       _next_trgt_pose)))
  (:continuing_mission
   (&optional (__continuing_mission :null))
   (if (not (eq __continuing_mission :null)) (setq _continuing_mission __continuing_mission)) _continuing_mission)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _next_trgt_pose
    (send _next_trgt_pose :serialization-length)
    ;; bool _continuing_mission
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _next_trgt_pose
       (send _next_trgt_pose :serialize s)
     ;; bool _continuing_mission
       (if _continuing_mission (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _next_trgt_pose
     (send _next_trgt_pose :deserialize buf ptr-) (incf ptr- (send _next_trgt_pose :serialization-length))
   ;; bool _continuing_mission
     (setq _continuing_mission (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass bboat_pkg::next_target_serv
  :super ros::object
  :slots ())

(setf (get bboat_pkg::next_target_serv :md5sum-) "d9ce4b52e25eaaef6e3a53a636b0b720")
(setf (get bboat_pkg::next_target_serv :datatype-) "bboat_pkg/next_target_serv")
(setf (get bboat_pkg::next_target_serv :request) bboat_pkg::next_target_servRequest)
(setf (get bboat_pkg::next_target_serv :response) bboat_pkg::next_target_servResponse)

(defmethod bboat_pkg::next_target_servRequest
  (:response () (instance bboat_pkg::next_target_servResponse :init)))

(setf (get bboat_pkg::next_target_servRequest :md5sum-) "d9ce4b52e25eaaef6e3a53a636b0b720")
(setf (get bboat_pkg::next_target_servRequest :datatype-) "bboat_pkg/next_target_servRequest")
(setf (get bboat_pkg::next_target_servRequest :definition-)
      "bool req
---
geometry_msgs/Pose next_trgt_pose
bool continuing_mission

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get bboat_pkg::next_target_servResponse :md5sum-) "d9ce4b52e25eaaef6e3a53a636b0b720")
(setf (get bboat_pkg::next_target_servResponse :datatype-) "bboat_pkg/next_target_servResponse")
(setf (get bboat_pkg::next_target_servResponse :definition-)
      "bool req
---
geometry_msgs/Pose next_trgt_pose
bool continuing_mission

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :bboat_pkg/next_target_serv "d9ce4b52e25eaaef6e3a53a636b0b720")


