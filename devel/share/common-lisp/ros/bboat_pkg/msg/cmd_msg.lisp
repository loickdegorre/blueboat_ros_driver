; Auto-generated. Do not edit!


(cl:in-package bboat_pkg-msg)


;//! \htmlinclude cmd_msg.msg.html

(cl:defclass <cmd_msg> (roslisp-msg-protocol:ros-message)
  ((u1
    :reader u1
    :initarg :u1
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (u2
    :reader u2
    :initarg :u2
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass cmd_msg (<cmd_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmd_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmd_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-msg:<cmd_msg> is deprecated: use bboat_pkg-msg:cmd_msg instead.")))

(cl:ensure-generic-function 'u1-val :lambda-list '(m))
(cl:defmethod u1-val ((m <cmd_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-msg:u1-val is deprecated.  Use bboat_pkg-msg:u1 instead.")
  (u1 m))

(cl:ensure-generic-function 'u2-val :lambda-list '(m))
(cl:defmethod u2-val ((m <cmd_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-msg:u2-val is deprecated.  Use bboat_pkg-msg:u2 instead.")
  (u2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmd_msg>) ostream)
  "Serializes a message object of type '<cmd_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'u1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'u2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmd_msg>) istream)
  "Deserializes a message object of type '<cmd_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'u1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'u2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmd_msg>)))
  "Returns string type for a message object of type '<cmd_msg>"
  "bboat_pkg/cmd_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_msg)))
  "Returns string type for a message object of type 'cmd_msg"
  "bboat_pkg/cmd_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmd_msg>)))
  "Returns md5sum for a message object of type '<cmd_msg>"
  "565f2132c1adf413c58d2ae5bf8d97b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmd_msg)))
  "Returns md5sum for a message object of type 'cmd_msg"
  "565f2132c1adf413c58d2ae5bf8d97b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmd_msg>)))
  "Returns full string definition for message of type '<cmd_msg>"
  (cl:format cl:nil "std_msgs/Float64 u1~%std_msgs/Float64 u2~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmd_msg)))
  "Returns full string definition for message of type 'cmd_msg"
  (cl:format cl:nil "std_msgs/Float64 u1~%std_msgs/Float64 u2~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmd_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'u1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'u2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmd_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'cmd_msg
    (cl:cons ':u1 (u1 msg))
    (cl:cons ':u2 (u2 msg))
))
