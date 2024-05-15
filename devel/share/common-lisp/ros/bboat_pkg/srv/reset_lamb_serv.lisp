; Auto-generated. Do not edit!


(cl:in-package bboat_pkg-srv)


;//! \htmlinclude reset_lamb_serv-request.msg.html

(cl:defclass <reset_lamb_serv-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass reset_lamb_serv-request (<reset_lamb_serv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <reset_lamb_serv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'reset_lamb_serv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-srv:<reset_lamb_serv-request> is deprecated: use bboat_pkg-srv:reset_lamb_serv-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <reset_lamb_serv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:req-val is deprecated.  Use bboat_pkg-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <reset_lamb_serv-request>) ostream)
  "Serializes a message object of type '<reset_lamb_serv-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'req) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <reset_lamb_serv-request>) istream)
  "Deserializes a message object of type '<reset_lamb_serv-request>"
    (cl:setf (cl:slot-value msg 'req) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<reset_lamb_serv-request>)))
  "Returns string type for a service object of type '<reset_lamb_serv-request>"
  "bboat_pkg/reset_lamb_servRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'reset_lamb_serv-request)))
  "Returns string type for a service object of type 'reset_lamb_serv-request"
  "bboat_pkg/reset_lamb_servRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<reset_lamb_serv-request>)))
  "Returns md5sum for a message object of type '<reset_lamb_serv-request>"
  "e188e0aaebb633ed14c21333921ef038")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'reset_lamb_serv-request)))
  "Returns md5sum for a message object of type 'reset_lamb_serv-request"
  "e188e0aaebb633ed14c21333921ef038")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<reset_lamb_serv-request>)))
  "Returns full string definition for message of type '<reset_lamb_serv-request>"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'reset_lamb_serv-request)))
  "Returns full string definition for message of type 'reset_lamb_serv-request"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <reset_lamb_serv-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <reset_lamb_serv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'reset_lamb_serv-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude reset_lamb_serv-response.msg.html

(cl:defclass <reset_lamb_serv-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass reset_lamb_serv-response (<reset_lamb_serv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <reset_lamb_serv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'reset_lamb_serv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-srv:<reset_lamb_serv-response> is deprecated: use bboat_pkg-srv:reset_lamb_serv-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <reset_lamb_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:success-val is deprecated.  Use bboat_pkg-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <reset_lamb_serv-response>) ostream)
  "Serializes a message object of type '<reset_lamb_serv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <reset_lamb_serv-response>) istream)
  "Deserializes a message object of type '<reset_lamb_serv-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<reset_lamb_serv-response>)))
  "Returns string type for a service object of type '<reset_lamb_serv-response>"
  "bboat_pkg/reset_lamb_servResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'reset_lamb_serv-response)))
  "Returns string type for a service object of type 'reset_lamb_serv-response"
  "bboat_pkg/reset_lamb_servResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<reset_lamb_serv-response>)))
  "Returns md5sum for a message object of type '<reset_lamb_serv-response>"
  "e188e0aaebb633ed14c21333921ef038")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'reset_lamb_serv-response)))
  "Returns md5sum for a message object of type 'reset_lamb_serv-response"
  "e188e0aaebb633ed14c21333921ef038")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<reset_lamb_serv-response>)))
  "Returns full string definition for message of type '<reset_lamb_serv-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'reset_lamb_serv-response)))
  "Returns full string definition for message of type 'reset_lamb_serv-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <reset_lamb_serv-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <reset_lamb_serv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'reset_lamb_serv-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'reset_lamb_serv)))
  'reset_lamb_serv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'reset_lamb_serv)))
  'reset_lamb_serv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'reset_lamb_serv)))
  "Returns string type for a service object of type '<reset_lamb_serv>"
  "bboat_pkg/reset_lamb_serv")