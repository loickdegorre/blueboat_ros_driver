; Auto-generated. Do not edit!


(cl:in-package bboat_pkg-srv)


;//! \htmlinclude lambert_ref_serv-request.msg.html

(cl:defclass <lambert_ref_serv-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass lambert_ref_serv-request (<lambert_ref_serv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lambert_ref_serv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lambert_ref_serv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-srv:<lambert_ref_serv-request> is deprecated: use bboat_pkg-srv:lambert_ref_serv-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <lambert_ref_serv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:req-val is deprecated.  Use bboat_pkg-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lambert_ref_serv-request>) ostream)
  "Serializes a message object of type '<lambert_ref_serv-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'req) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lambert_ref_serv-request>) istream)
  "Deserializes a message object of type '<lambert_ref_serv-request>"
    (cl:setf (cl:slot-value msg 'req) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lambert_ref_serv-request>)))
  "Returns string type for a service object of type '<lambert_ref_serv-request>"
  "bboat_pkg/lambert_ref_servRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lambert_ref_serv-request)))
  "Returns string type for a service object of type 'lambert_ref_serv-request"
  "bboat_pkg/lambert_ref_servRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lambert_ref_serv-request>)))
  "Returns md5sum for a message object of type '<lambert_ref_serv-request>"
  "2f9744e749801011d7a2f5581e48ef29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lambert_ref_serv-request)))
  "Returns md5sum for a message object of type 'lambert_ref_serv-request"
  "2f9744e749801011d7a2f5581e48ef29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lambert_ref_serv-request>)))
  "Returns full string definition for message of type '<lambert_ref_serv-request>"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lambert_ref_serv-request)))
  "Returns full string definition for message of type 'lambert_ref_serv-request"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lambert_ref_serv-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lambert_ref_serv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'lambert_ref_serv-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude lambert_ref_serv-response.msg.html

(cl:defclass <lambert_ref_serv-response> (roslisp-msg-protocol:ros-message)
  ((lambert_ref
    :reader lambert_ref
    :initarg :lambert_ref
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass lambert_ref_serv-response (<lambert_ref_serv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lambert_ref_serv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lambert_ref_serv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-srv:<lambert_ref_serv-response> is deprecated: use bboat_pkg-srv:lambert_ref_serv-response instead.")))

(cl:ensure-generic-function 'lambert_ref-val :lambda-list '(m))
(cl:defmethod lambert_ref-val ((m <lambert_ref_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:lambert_ref-val is deprecated.  Use bboat_pkg-srv:lambert_ref instead.")
  (lambert_ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lambert_ref_serv-response>) ostream)
  "Serializes a message object of type '<lambert_ref_serv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'lambert_ref) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lambert_ref_serv-response>) istream)
  "Deserializes a message object of type '<lambert_ref_serv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'lambert_ref) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lambert_ref_serv-response>)))
  "Returns string type for a service object of type '<lambert_ref_serv-response>"
  "bboat_pkg/lambert_ref_servResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lambert_ref_serv-response)))
  "Returns string type for a service object of type 'lambert_ref_serv-response"
  "bboat_pkg/lambert_ref_servResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lambert_ref_serv-response>)))
  "Returns md5sum for a message object of type '<lambert_ref_serv-response>"
  "2f9744e749801011d7a2f5581e48ef29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lambert_ref_serv-response)))
  "Returns md5sum for a message object of type 'lambert_ref_serv-response"
  "2f9744e749801011d7a2f5581e48ef29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lambert_ref_serv-response>)))
  "Returns full string definition for message of type '<lambert_ref_serv-response>"
  (cl:format cl:nil "geometry_msgs/Point lambert_ref~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lambert_ref_serv-response)))
  "Returns full string definition for message of type 'lambert_ref_serv-response"
  (cl:format cl:nil "geometry_msgs/Point lambert_ref~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lambert_ref_serv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'lambert_ref))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lambert_ref_serv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'lambert_ref_serv-response
    (cl:cons ':lambert_ref (lambert_ref msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'lambert_ref_serv)))
  'lambert_ref_serv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'lambert_ref_serv)))
  'lambert_ref_serv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lambert_ref_serv)))
  "Returns string type for a service object of type '<lambert_ref_serv>"
  "bboat_pkg/lambert_ref_serv")