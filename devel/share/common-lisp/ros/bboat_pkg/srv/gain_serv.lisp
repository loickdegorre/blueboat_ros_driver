; Auto-generated. Do not edit!


(cl:in-package bboat_pkg-srv)


;//! \htmlinclude gain_serv-request.msg.html

(cl:defclass <gain_serv-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass gain_serv-request (<gain_serv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gain_serv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gain_serv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-srv:<gain_serv-request> is deprecated: use bboat_pkg-srv:gain_serv-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <gain_serv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:req-val is deprecated.  Use bboat_pkg-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gain_serv-request>) ostream)
  "Serializes a message object of type '<gain_serv-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'req) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gain_serv-request>) istream)
  "Deserializes a message object of type '<gain_serv-request>"
    (cl:setf (cl:slot-value msg 'req) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gain_serv-request>)))
  "Returns string type for a service object of type '<gain_serv-request>"
  "bboat_pkg/gain_servRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gain_serv-request)))
  "Returns string type for a service object of type 'gain_serv-request"
  "bboat_pkg/gain_servRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gain_serv-request>)))
  "Returns md5sum for a message object of type '<gain_serv-request>"
  "5f632b8cb09f5ccd1ae01e67f49049ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gain_serv-request)))
  "Returns md5sum for a message object of type 'gain_serv-request"
  "5f632b8cb09f5ccd1ae01e67f49049ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gain_serv-request>)))
  "Returns full string definition for message of type '<gain_serv-request>"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gain_serv-request)))
  "Returns full string definition for message of type 'gain_serv-request"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gain_serv-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gain_serv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gain_serv-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude gain_serv-response.msg.html

(cl:defclass <gain_serv-response> (roslisp-msg-protocol:ros-message)
  ((kp_1
    :reader kp_1
    :initarg :kp_1
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (ki_1
    :reader ki_1
    :initarg :ki_1
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (kd_1
    :reader kd_1
    :initarg :kd_1
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (kp_2
    :reader kp_2
    :initarg :kp_2
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (ki_2
    :reader ki_2
    :initarg :ki_2
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (kd_2
    :reader kd_2
    :initarg :kd_2
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass gain_serv-response (<gain_serv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gain_serv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gain_serv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-srv:<gain_serv-response> is deprecated: use bboat_pkg-srv:gain_serv-response instead.")))

(cl:ensure-generic-function 'kp_1-val :lambda-list '(m))
(cl:defmethod kp_1-val ((m <gain_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:kp_1-val is deprecated.  Use bboat_pkg-srv:kp_1 instead.")
  (kp_1 m))

(cl:ensure-generic-function 'ki_1-val :lambda-list '(m))
(cl:defmethod ki_1-val ((m <gain_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:ki_1-val is deprecated.  Use bboat_pkg-srv:ki_1 instead.")
  (ki_1 m))

(cl:ensure-generic-function 'kd_1-val :lambda-list '(m))
(cl:defmethod kd_1-val ((m <gain_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:kd_1-val is deprecated.  Use bboat_pkg-srv:kd_1 instead.")
  (kd_1 m))

(cl:ensure-generic-function 'kp_2-val :lambda-list '(m))
(cl:defmethod kp_2-val ((m <gain_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:kp_2-val is deprecated.  Use bboat_pkg-srv:kp_2 instead.")
  (kp_2 m))

(cl:ensure-generic-function 'ki_2-val :lambda-list '(m))
(cl:defmethod ki_2-val ((m <gain_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:ki_2-val is deprecated.  Use bboat_pkg-srv:ki_2 instead.")
  (ki_2 m))

(cl:ensure-generic-function 'kd_2-val :lambda-list '(m))
(cl:defmethod kd_2-val ((m <gain_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:kd_2-val is deprecated.  Use bboat_pkg-srv:kd_2 instead.")
  (kd_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gain_serv-response>) ostream)
  "Serializes a message object of type '<gain_serv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'kp_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ki_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'kd_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'kp_2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ki_2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'kd_2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gain_serv-response>) istream)
  "Deserializes a message object of type '<gain_serv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'kp_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ki_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'kd_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'kp_2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ki_2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'kd_2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gain_serv-response>)))
  "Returns string type for a service object of type '<gain_serv-response>"
  "bboat_pkg/gain_servResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gain_serv-response)))
  "Returns string type for a service object of type 'gain_serv-response"
  "bboat_pkg/gain_servResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gain_serv-response>)))
  "Returns md5sum for a message object of type '<gain_serv-response>"
  "5f632b8cb09f5ccd1ae01e67f49049ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gain_serv-response)))
  "Returns md5sum for a message object of type 'gain_serv-response"
  "5f632b8cb09f5ccd1ae01e67f49049ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gain_serv-response>)))
  "Returns full string definition for message of type '<gain_serv-response>"
  (cl:format cl:nil "std_msgs/Float64 kp_1~%std_msgs/Float64 ki_1~%std_msgs/Float64 kd_1~%std_msgs/Float64 kp_2~%std_msgs/Float64 ki_2~%std_msgs/Float64 kd_2~%~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gain_serv-response)))
  "Returns full string definition for message of type 'gain_serv-response"
  (cl:format cl:nil "std_msgs/Float64 kp_1~%std_msgs/Float64 ki_1~%std_msgs/Float64 kd_1~%std_msgs/Float64 kp_2~%std_msgs/Float64 ki_2~%std_msgs/Float64 kd_2~%~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gain_serv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'kp_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ki_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'kd_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'kp_2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ki_2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'kd_2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gain_serv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gain_serv-response
    (cl:cons ':kp_1 (kp_1 msg))
    (cl:cons ':ki_1 (ki_1 msg))
    (cl:cons ':kd_1 (kd_1 msg))
    (cl:cons ':kp_2 (kp_2 msg))
    (cl:cons ':ki_2 (ki_2 msg))
    (cl:cons ':kd_2 (kd_2 msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gain_serv)))
  'gain_serv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gain_serv)))
  'gain_serv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gain_serv)))
  "Returns string type for a service object of type '<gain_serv>"
  "bboat_pkg/gain_serv")