;; Auto-generated. Do not edit!


(when (boundp 'bboat_pkg::lambert_ref_serv)
  (if (not (find-package "BBOAT_PKG"))
    (make-package "BBOAT_PKG"))
  (shadow 'lambert_ref_serv (find-package "BBOAT_PKG")))
(unless (find-package "BBOAT_PKG::LAMBERT_REF_SERV")
  (make-package "BBOAT_PKG::LAMBERT_REF_SERV"))
(unless (find-package "BBOAT_PKG::LAMBERT_REF_SERVREQUEST")
  (make-package "BBOAT_PKG::LAMBERT_REF_SERVREQUEST"))
(unless (find-package "BBOAT_PKG::LAMBERT_REF_SERVRESPONSE")
  (make-package "BBOAT_PKG::LAMBERT_REF_SERVRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass bboat_pkg::lambert_ref_servRequest
  :super ros::object
  :slots (_req ))

(defmethod bboat_pkg::lambert_ref_servRequest
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

(defclass bboat_pkg::lambert_ref_servResponse
  :super ros::object
  :slots (_lambert_ref ))

(defmethod bboat_pkg::lambert_ref_servResponse
  (:init
   (&key
    ((:lambert_ref __lambert_ref) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _lambert_ref __lambert_ref)
   self)
  (:lambert_ref
   (&rest __lambert_ref)
   (if (keywordp (car __lambert_ref))
       (send* _lambert_ref __lambert_ref)
     (progn
       (if __lambert_ref (setq _lambert_ref (car __lambert_ref)))
       _lambert_ref)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _lambert_ref
    (send _lambert_ref :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _lambert_ref
       (send _lambert_ref :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _lambert_ref
     (send _lambert_ref :deserialize buf ptr-) (incf ptr- (send _lambert_ref :serialization-length))
   ;;
   self)
  )

(defclass bboat_pkg::lambert_ref_serv
  :super ros::object
  :slots ())

(setf (get bboat_pkg::lambert_ref_serv :md5sum-) "2f9744e749801011d7a2f5581e48ef29")
(setf (get bboat_pkg::lambert_ref_serv :datatype-) "bboat_pkg/lambert_ref_serv")
(setf (get bboat_pkg::lambert_ref_serv :request) bboat_pkg::lambert_ref_servRequest)
(setf (get bboat_pkg::lambert_ref_serv :response) bboat_pkg::lambert_ref_servResponse)

(defmethod bboat_pkg::lambert_ref_servRequest
  (:response () (instance bboat_pkg::lambert_ref_servResponse :init)))

(setf (get bboat_pkg::lambert_ref_servRequest :md5sum-) "2f9744e749801011d7a2f5581e48ef29")
(setf (get bboat_pkg::lambert_ref_servRequest :datatype-) "bboat_pkg/lambert_ref_servRequest")
(setf (get bboat_pkg::lambert_ref_servRequest :definition-)
      "bool req
---
geometry_msgs/Point lambert_ref


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get bboat_pkg::lambert_ref_servResponse :md5sum-) "2f9744e749801011d7a2f5581e48ef29")
(setf (get bboat_pkg::lambert_ref_servResponse :datatype-) "bboat_pkg/lambert_ref_servResponse")
(setf (get bboat_pkg::lambert_ref_servResponse :definition-)
      "bool req
---
geometry_msgs/Point lambert_ref


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :bboat_pkg/lambert_ref_serv "2f9744e749801011d7a2f5581e48ef29")


