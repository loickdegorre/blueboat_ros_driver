;; Auto-generated. Do not edit!


(when (boundp 'bboat_pkg::cmd_msg)
  (if (not (find-package "BBOAT_PKG"))
    (make-package "BBOAT_PKG"))
  (shadow 'cmd_msg (find-package "BBOAT_PKG")))
(unless (find-package "BBOAT_PKG::CMD_MSG")
  (make-package "BBOAT_PKG::CMD_MSG"))

(in-package "ROS")
;;//! \htmlinclude cmd_msg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass bboat_pkg::cmd_msg
  :super ros::object
  :slots (_u1 _u2 ))

(defmethod bboat_pkg::cmd_msg
  (:init
   (&key
    ((:u1 __u1) (instance std_msgs::Float64 :init))
    ((:u2 __u2) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _u1 __u1)
   (setq _u2 __u2)
   self)
  (:u1
   (&rest __u1)
   (if (keywordp (car __u1))
       (send* _u1 __u1)
     (progn
       (if __u1 (setq _u1 (car __u1)))
       _u1)))
  (:u2
   (&rest __u2)
   (if (keywordp (car __u2))
       (send* _u2 __u2)
     (progn
       (if __u2 (setq _u2 (car __u2)))
       _u2)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64 _u1
    (send _u1 :serialization-length)
    ;; std_msgs/Float64 _u2
    (send _u2 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64 _u1
       (send _u1 :serialize s)
     ;; std_msgs/Float64 _u2
       (send _u2 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64 _u1
     (send _u1 :deserialize buf ptr-) (incf ptr- (send _u1 :serialization-length))
   ;; std_msgs/Float64 _u2
     (send _u2 :deserialize buf ptr-) (incf ptr- (send _u2 :serialization-length))
   ;;
   self)
  )

(setf (get bboat_pkg::cmd_msg :md5sum-) "565f2132c1adf413c58d2ae5bf8d97b9")
(setf (get bboat_pkg::cmd_msg :datatype-) "bboat_pkg/cmd_msg")
(setf (get bboat_pkg::cmd_msg :definition-)
      "std_msgs/Float64 u1
std_msgs/Float64 u2

================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :bboat_pkg/cmd_msg "565f2132c1adf413c58d2ae5bf8d97b9")


