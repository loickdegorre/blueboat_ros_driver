;; Auto-generated. Do not edit!


(when (boundp 'bboat_pkg::current_target_serv)
  (if (not (find-package "BBOAT_PKG"))
    (make-package "BBOAT_PKG"))
  (shadow 'current_target_serv (find-package "BBOAT_PKG")))
(unless (find-package "BBOAT_PKG::CURRENT_TARGET_SERV")
  (make-package "BBOAT_PKG::CURRENT_TARGET_SERV"))
(unless (find-package "BBOAT_PKG::CURRENT_TARGET_SERVREQUEST")
  (make-package "BBOAT_PKG::CURRENT_TARGET_SERVREQUEST"))
(unless (find-package "BBOAT_PKG::CURRENT_TARGET_SERVRESPONSE")
  (make-package "BBOAT_PKG::CURRENT_TARGET_SERVRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass bboat_pkg::current_target_servRequest
  :super ros::object
  :slots (_req ))

(defmethod bboat_pkg::current_target_servRequest
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

(defclass bboat_pkg::current_target_servResponse
  :super ros::object
  :slots (_target ))

(defmethod bboat_pkg::current_target_servResponse
  (:init
   (&key
    ((:target __target) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _target __target)
   self)
  (:target
   (&rest __target)
   (if (keywordp (car __target))
       (send* _target __target)
     (progn
       (if __target (setq _target (car __target)))
       _target)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _target
    (send _target :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _target
       (send _target :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _target
     (send _target :deserialize buf ptr-) (incf ptr- (send _target :serialization-length))
   ;;
   self)
  )

(defclass bboat_pkg::current_target_serv
  :super ros::object
  :slots ())

(setf (get bboat_pkg::current_target_serv :md5sum-) "a9f2a4511e92af8f0b1ba9bcfcaed70e")
(setf (get bboat_pkg::current_target_serv :datatype-) "bboat_pkg/current_target_serv")
(setf (get bboat_pkg::current_target_serv :request) bboat_pkg::current_target_servRequest)
(setf (get bboat_pkg::current_target_serv :response) bboat_pkg::current_target_servResponse)

(defmethod bboat_pkg::current_target_servRequest
  (:response () (instance bboat_pkg::current_target_servResponse :init)))

(setf (get bboat_pkg::current_target_servRequest :md5sum-) "a9f2a4511e92af8f0b1ba9bcfcaed70e")
(setf (get bboat_pkg::current_target_servRequest :datatype-) "bboat_pkg/current_target_servRequest")
(setf (get bboat_pkg::current_target_servRequest :definition-)
      "bool req
---
geometry_msgs/Point target


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get bboat_pkg::current_target_servResponse :md5sum-) "a9f2a4511e92af8f0b1ba9bcfcaed70e")
(setf (get bboat_pkg::current_target_servResponse :datatype-) "bboat_pkg/current_target_servResponse")
(setf (get bboat_pkg::current_target_servResponse :definition-)
      "bool req
---
geometry_msgs/Point target


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :bboat_pkg/current_target_serv "a9f2a4511e92af8f0b1ba9bcfcaed70e")


