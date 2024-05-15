; Auto-generated. Do not edit!


(cl:in-package bboat_pkg-srv)


;//! \htmlinclude mode_serv-request.msg.html

(cl:defclass <mode_serv-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass mode_serv-request (<mode_serv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mode_serv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mode_serv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-srv:<mode_serv-request> is deprecated: use bboat_pkg-srv:mode_serv-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <mode_serv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:req-val is deprecated.  Use bboat_pkg-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mode_serv-request>) ostream)
  "Serializes a message object of type '<mode_serv-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'req) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mode_serv-request>) istream)
  "Deserializes a message object of type '<mode_serv-request>"
    (cl:setf (cl:slot-value msg 'req) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mode_serv-request>)))
  "Returns string type for a service object of type '<mode_serv-request>"
  "bboat_pkg/mode_servRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mode_serv-request)))
  "Returns string type for a service object of type 'mode_serv-request"
  "bboat_pkg/mode_servRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mode_serv-request>)))
  "Returns md5sum for a message object of type '<mode_serv-request>"
  "a6ec36370aeacdd14ee457803ad5dd8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mode_serv-request)))
  "Returns md5sum for a message object of type 'mode_serv-request"
  "a6ec36370aeacdd14ee457803ad5dd8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mode_serv-request>)))
  "Returns full string definition for message of type '<mode_serv-request>"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mode_serv-request)))
  "Returns full string definition for message of type 'mode_serv-request"
  (cl:format cl:nil "bool req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mode_serv-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mode_serv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'mode_serv-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude mode_serv-response.msg.html

(cl:defclass <mode_serv-response> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:string
    :initform "")
   (mission
    :reader mission
    :initarg :mission
    :type cl:string
    :initform ""))
)

(cl:defclass mode_serv-response (<mode_serv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mode_serv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mode_serv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-srv:<mode_serv-response> is deprecated: use bboat_pkg-srv:mode_serv-response instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <mode_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:mode-val is deprecated.  Use bboat_pkg-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'mission-val :lambda-list '(m))
(cl:defmethod mission-val ((m <mode_serv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-srv:mission-val is deprecated.  Use bboat_pkg-srv:mission instead.")
  (mission m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mode_serv-response>) ostream)
  "Serializes a message object of type '<mode_serv-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mission))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mission))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mode_serv-response>) istream)
  "Deserializes a message object of type '<mode_serv-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mission) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mission) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mode_serv-response>)))
  "Returns string type for a service object of type '<mode_serv-response>"
  "bboat_pkg/mode_servResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mode_serv-response)))
  "Returns string type for a service object of type 'mode_serv-response"
  "bboat_pkg/mode_servResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mode_serv-response>)))
  "Returns md5sum for a message object of type '<mode_serv-response>"
  "a6ec36370aeacdd14ee457803ad5dd8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mode_serv-response)))
  "Returns md5sum for a message object of type 'mode_serv-response"
  "a6ec36370aeacdd14ee457803ad5dd8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mode_serv-response>)))
  "Returns full string definition for message of type '<mode_serv-response>"
  (cl:format cl:nil "string mode~%string mission~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mode_serv-response)))
  "Returns full string definition for message of type 'mode_serv-response"
  (cl:format cl:nil "string mode~%string mission~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mode_serv-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'mode))
     4 (cl:length (cl:slot-value msg 'mission))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mode_serv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'mode_serv-response
    (cl:cons ':mode (mode msg))
    (cl:cons ':mission (mission msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'mode_serv)))
  'mode_serv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'mode_serv)))
  'mode_serv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mode_serv)))
  "Returns string type for a service object of type '<mode_serv>"
  "bboat_pkg/mode_serv")