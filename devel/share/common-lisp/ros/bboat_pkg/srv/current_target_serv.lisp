; Auto-generated. Do not edit!


(cl:in-package bboat_pkg-srv)


;//! \htmlinclude current_target_serv-request.msg.html

(cl:defclass <current_target_serv-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass current_target_serv-request (<current_target_serv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <current_target_serv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'current_target_serv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-srv:<current_target_serv-request> is deprecated: use bboat_pkg-srv:current_target_serv-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <current_target_serv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:req-val is deprecated.  Use bboat_pkg-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <current_target_serv-request>) ostream)
  "Serializes a message object of type '<current_target_serv-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'req) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <current_target_serv-request>) istream)
  "Deserializes a message object of type '<current_target_serv-request>"
    (cl:setf (cl:slot-value msg 'req) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<current_target_serv-request>)))
  "Returns string type for a service object of type '<current_target_serv-request>"
  "bboat_pkg/current_target_servRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'current_target_serv-request)))
  "Returns string type for a service object of type 'current_target_serv-request"
  "bboat_pkg/current_target_servRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<current_target_serv-request>)))
  "Returns md5sum for a message object of type '<current_target_serv-request>"
  "a9f2a4511e92af8f0b1ba9bcfcaed70e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'current_target_serv-request)))
  "Returns md5sum for a message object of type 'current_target_serv-request"
  "a9f2a4511e92af8f0b1ba9bcfcaed70e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<current_target_serv-request>)))
  "Returns full string definition for message of type '<current_target_serv-request>"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'current_target_serv-request)))
  "Returns full string definition for message of type 'current_target_serv-request"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <current_target_serv-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <current_target_serv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'current_target_serv-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude current_target_serv-response.msg.html

(cl:defclass <current_target_serv-response> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass current_target_serv-response (<current_target_serv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <current_target_serv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'current_target_serv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-srv:<current_target_serv-response> is deprecated: use bboat_pkg-srv:current_target_serv-response instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <current_target_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:target-val is deprecated.  Use bboat_pkg-srv:target instead.")
  (target m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <current_target_serv-response>) ostream)
  "Serializes a message object of type '<current_target_serv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <current_target_serv-response>) istream)
  "Deserializes a message object of type '<current_target_serv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<current_target_serv-response>)))
  "Returns string type for a service object of type '<current_target_serv-response>"
  "bboat_pkg/current_target_servResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'current_target_serv-response)))
  "Returns string type for a service object of type 'current_target_serv-response"
  "bboat_pkg/current_target_servResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<current_target_serv-response>)))
  "Returns md5sum for a message object of type '<current_target_serv-response>"
  "a9f2a4511e92af8f0b1ba9bcfcaed70e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'current_target_serv-response)))
  "Returns md5sum for a message object of type 'current_target_serv-response"
  "a9f2a4511e92af8f0b1ba9bcfcaed70e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<current_target_serv-response>)))
  "Returns full string definition for message of type '<current_target_serv-response>"
  (cl:format cl:nil "geometry_msgs/Point target~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'current_target_serv-response)))
  "Returns full string definition for message of type 'current_target_serv-response"
  (cl:format cl:nil "geometry_msgs/Point target~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <current_target_serv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <current_target_serv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'current_target_serv-response
    (cl:cons ':target (target msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'current_target_serv)))
  'current_target_serv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'current_target_serv)))
  'current_target_serv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'current_target_serv)))
  "Returns string type for a service object of type '<current_target_serv>"
  "bboat_pkg/current_target_serv")