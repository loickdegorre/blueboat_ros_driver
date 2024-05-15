; Auto-generated. Do not edit!


(cl:in-package bboat_pkg-srv)


;//! \htmlinclude next_target_serv-request.msg.html

(cl:defclass <next_target_serv-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass next_target_serv-request (<next_target_serv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <next_target_serv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'next_target_serv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-srv:<next_target_serv-request> is deprecated: use bboat_pkg-srv:next_target_serv-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <next_target_serv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:req-val is deprecated.  Use bboat_pkg-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <next_target_serv-request>) ostream)
  "Serializes a message object of type '<next_target_serv-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'req) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <next_target_serv-request>) istream)
  "Deserializes a message object of type '<next_target_serv-request>"
    (cl:setf (cl:slot-value msg 'req) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<next_target_serv-request>)))
  "Returns string type for a service object of type '<next_target_serv-request>"
  "bboat_pkg/next_target_servRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'next_target_serv-request)))
  "Returns string type for a service object of type 'next_target_serv-request"
  "bboat_pkg/next_target_servRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<next_target_serv-request>)))
  "Returns md5sum for a message object of type '<next_target_serv-request>"
  "d9ce4b52e25eaaef6e3a53a636b0b720")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'next_target_serv-request)))
  "Returns md5sum for a message object of type 'next_target_serv-request"
  "d9ce4b52e25eaaef6e3a53a636b0b720")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<next_target_serv-request>)))
  "Returns full string definition for message of type '<next_target_serv-request>"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'next_target_serv-request)))
  "Returns full string definition for message of type 'next_target_serv-request"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <next_target_serv-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <next_target_serv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'next_target_serv-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude next_target_serv-response.msg.html

(cl:defclass <next_target_serv-response> (roslisp-msg-protocol:ros-message)
  ((next_trgt_pose
    :reader next_trgt_pose
    :initarg :next_trgt_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (continuing_mission
    :reader continuing_mission
    :initarg :continuing_mission
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass next_target_serv-response (<next_target_serv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <next_target_serv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'next_target_serv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-srv:<next_target_serv-response> is deprecated: use bboat_pkg-srv:next_target_serv-response instead.")))

(cl:ensure-generic-function 'next_trgt_pose-val :lambda-list '(m))
(cl:defmethod next_trgt_pose-val ((m <next_target_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:next_trgt_pose-val is deprecated.  Use bboat_pkg-srv:next_trgt_pose instead.")
  (next_trgt_pose m))

(cl:ensure-generic-function 'continuing_mission-val :lambda-list '(m))
(cl:defmethod continuing_mission-val ((m <next_target_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:continuing_mission-val is deprecated.  Use bboat_pkg-srv:continuing_mission instead.")
  (continuing_mission m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <next_target_serv-response>) ostream)
  "Serializes a message object of type '<next_target_serv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'next_trgt_pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'continuing_mission) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <next_target_serv-response>) istream)
  "Deserializes a message object of type '<next_target_serv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'next_trgt_pose) istream)
    (cl:setf (cl:slot-value msg 'continuing_mission) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<next_target_serv-response>)))
  "Returns string type for a service object of type '<next_target_serv-response>"
  "bboat_pkg/next_target_servResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'next_target_serv-response)))
  "Returns string type for a service object of type 'next_target_serv-response"
  "bboat_pkg/next_target_servResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<next_target_serv-response>)))
  "Returns md5sum for a message object of type '<next_target_serv-response>"
  "d9ce4b52e25eaaef6e3a53a636b0b720")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'next_target_serv-response)))
  "Returns md5sum for a message object of type 'next_target_serv-response"
  "d9ce4b52e25eaaef6e3a53a636b0b720")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<next_target_serv-response>)))
  "Returns full string definition for message of type '<next_target_serv-response>"
  (cl:format cl:nil "geometry_msgs/Pose next_trgt_pose~%bool continuing_mission~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'next_target_serv-response)))
  "Returns full string definition for message of type 'next_target_serv-response"
  (cl:format cl:nil "geometry_msgs/Pose next_trgt_pose~%bool continuing_mission~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <next_target_serv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'next_trgt_pose))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <next_target_serv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'next_target_serv-response
    (cl:cons ':next_trgt_pose (next_trgt_pose msg))
    (cl:cons ':continuing_mission (continuing_mission msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'next_target_serv)))
  'next_target_serv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'next_target_serv)))
  'next_target_serv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'next_target_serv)))
  "Returns string type for a service object of type '<next_target_serv>"
  "bboat_pkg/next_target_serv")