;; Auto-generated. Do not edit!


(when (boundp 'bboat_pkg::gain_serv)
  (if (not (find-package "BBOAT_PKG"))
    (make-package "BBOAT_PKG"))
  (shadow 'gain_serv (find-package "BBOAT_PKG")))
(unless (find-package "BBOAT_PKG::GAIN_SERV")
  (make-package "BBOAT_PKG::GAIN_SERV"))
(unless (find-package "BBOAT_PKG::GAIN_SERVREQUEST")
  (make-package "BBOAT_PKG::GAIN_SERVREQUEST"))
(unless (find-package "BBOAT_PKG::GAIN_SERVRESPONSE")
  (make-package "BBOAT_PKG::GAIN_SERVRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass bboat_pkg::gain_servRequest
  :super ros::object
  :slots (_req ))

(defmethod bboat_pkg::gain_servRequest
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

(defclass bboat_pkg::gain_servResponse
  :super ros::object
  :slots (_kp_1 _ki_1 _kd_1 _kp_2 _ki_2 _kd_2 ))

(defmethod bboat_pkg::gain_servResponse
  (:init
   (&key
    ((:kp_1 __kp_1) (instance std_msgs::Float64 :init))
    ((:ki_1 __ki_1) (instance std_msgs::Float64 :init))
    ((:kd_1 __kd_1) (instance std_msgs::Float64 :init))
    ((:kp_2 __kp_2) (instance std_msgs::Float64 :init))
    ((:ki_2 __ki_2) (instance std_msgs::Float64 :init))
    ((:kd_2 __kd_2) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _kp_1 __kp_1)
   (setq _ki_1 __ki_1)
   (setq _kd_1 __kd_1)
   (setq _kp_2 __kp_2)
   (setq _ki_2 __ki_2)
   (setq _kd_2 __kd_2)
   self)
  (:kp_1
   (&rest __kp_1)
   (if (keywordp (car __kp_1))
       (send* _kp_1 __kp_1)
     (progn
       (if __kp_1 (setq _kp_1 (car __kp_1)))
       _kp_1)))
  (:ki_1
   (&rest __ki_1)
   (if (keywordp (car __ki_1))
       (send* _ki_1 __ki_1)
     (progn
       (if __ki_1 (setq _ki_1 (car __ki_1)))
       _ki_1)))
  (:kd_1
   (&rest __kd_1)
   (if (keywordp (car __kd_1))
       (send* _kd_1 __kd_1)
     (progn
       (if __kd_1 (setq _kd_1 (car __kd_1)))
       _kd_1)))
  (:kp_2
   (&rest __kp_2)
   (if (keywordp (car __kp_2))
       (send* _kp_2 __kp_2)
     (progn
       (if __kp_2 (setq _kp_2 (car __kp_2)))
       _kp_2)))
  (:ki_2
   (&rest __ki_2)
   (if (keywordp (car __ki_2))
       (send* _ki_2 __ki_2)
     (progn
       (if __ki_2 (setq _ki_2 (car __ki_2)))
       _ki_2)))
  (:kd_2
   (&rest __kd_2)
   (if (keywordp (car __kd_2))
       (send* _kd_2 __kd_2)
     (progn
       (if __kd_2 (setq _kd_2 (car __kd_2)))
       _kd_2)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64 _kp_1
    (send _kp_1 :serialization-length)
    ;; std_msgs/Float64 _ki_1
    (send _ki_1 :serialization-length)
    ;; std_msgs/Float64 _kd_1
    (send _kd_1 :serialization-length)
    ;; std_msgs/Float64 _kp_2
    (send _kp_2 :serialization-length)
    ;; std_msgs/Float64 _ki_2
    (send _ki_2 :serialization-length)
    ;; std_msgs/Float64 _kd_2
    (send _kd_2 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64 _kp_1
       (send _kp_1 :serialize s)
     ;; std_msgs/Float64 _ki_1
       (send _ki_1 :serialize s)
     ;; std_msgs/Float64 _kd_1
       (send _kd_1 :serialize s)
     ;; std_msgs/Float64 _kp_2
       (send _kp_2 :serialize s)
     ;; std_msgs/Float64 _ki_2
       (send _ki_2 :serialize s)
     ;; std_msgs/Float64 _kd_2
       (send _kd_2 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64 _kp_1
     (send _kp_1 :deserialize buf ptr-) (incf ptr- (send _kp_1 :serialization-length))
   ;; std_msgs/Float64 _ki_1
     (send _ki_1 :deserialize buf ptr-) (incf ptr- (send _ki_1 :serialization-length))
   ;; std_msgs/Float64 _kd_1
     (send _kd_1 :deserialize buf ptr-) (incf ptr- (send _kd_1 :serialization-length))
   ;; std_msgs/Float64 _kp_2
     (send _kp_2 :deserialize buf ptr-) (incf ptr- (send _kp_2 :serialization-length))
   ;; std_msgs/Float64 _ki_2
     (send _ki_2 :deserialize buf ptr-) (incf ptr- (send _ki_2 :serialization-length))
   ;; std_msgs/Float64 _kd_2
     (send _kd_2 :deserialize buf ptr-) (incf ptr- (send _kd_2 :serialization-length))
   ;;
   self)
  )

(defclass bboat_pkg::gain_serv
  :super ros::object
  :slots ())

(setf (get bboat_pkg::gain_serv :md5sum-) "5f632b8cb09f5ccd1ae01e67f49049ea")
(setf (get bboat_pkg::gain_serv :datatype-) "bboat_pkg/gain_serv")
(setf (get bboat_pkg::gain_serv :request) bboat_pkg::gain_servRequest)
(setf (get bboat_pkg::gain_serv :response) bboat_pkg::gain_servResponse)

(defmethod bboat_pkg::gain_servRequest
  (:response () (instance bboat_pkg::gain_servResponse :init)))

(setf (get bboat_pkg::gain_servRequest :md5sum-) "5f632b8cb09f5ccd1ae01e67f49049ea")
(setf (get bboat_pkg::gain_servRequest :datatype-) "bboat_pkg/gain_servRequest")
(setf (get bboat_pkg::gain_servRequest :definition-)
      "bool req
---
std_msgs/Float64 kp_1
std_msgs/Float64 ki_1
std_msgs/Float64 kd_1
std_msgs/Float64 kp_2
std_msgs/Float64 ki_2
std_msgs/Float64 kd_2


================================================================================
MSG: std_msgs/Float64
")

(setf (get bboat_pkg::gain_servResponse :md5sum-) "5f632b8cb09f5ccd1ae01e67f49049ea")
(setf (get bboat_pkg::gain_servResponse :datatype-) "bboat_pkg/gain_servResponse")
(setf (get bboat_pkg::gain_servResponse :definition-)
      "bool req
---
std_msgs/Float64 kp_1
std_msgs/Float64 ki_1
std_msgs/Float64 kd_1
std_msgs/Float64 kp_2
std_msgs/Float64 ki_2
std_msgs/Float64 kd_2


================================================================================
MSG: std_msgs/Float64
")



(provide :bboat_pkg/gain_serv "5f632b8cb09f5ccd1ae01e67f49049ea")


