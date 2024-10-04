; Auto-generated. Do not edit!


(cl:in-package bboat_pkg-msg)


;//! \htmlinclude mode_msg.msg.html

(cl:defclass <mode_msg> (roslisp-msg-protocol:ros-message)
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

(cl:defclass mode_msg (<mode_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mode_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mode_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bboat_pkg-msg:<mode_msg> is deprecated: use bboat_pkg-msg:mode_msg instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <mode_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-msg:mode-val is deprecated.  Use bboat_pkg-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'mission-val :lambda-list '(m))
(cl:defmethod mission-val ((m <mode_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bboat_pkg-msg:mission-val is deprecated.  Use bboat_pkg-msg:mission instead.")
  (mission m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mode_msg>) ostream)
  "Serializes a message object of type '<mode_msg>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mode_msg>) istream)
  "Deserializes a message object of type '<mode_msg>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mode_msg>)))
  "Returns string type for a message object of type '<mode_msg>"
  "bboat_pkg/mode_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mode_msg)))
  "Returns string type for a message object of type 'mode_msg"
  "bboat_pkg/mode_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mode_msg>)))
  "Returns md5sum for a message object of type '<mode_msg>"
  "6609a085032a3aa05e4e4147b4c8d4d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mode_msg)))
  "Returns md5sum for a message object of type 'mode_msg"
  "6609a085032a3aa05e4e4147b4c8d4d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mode_msg>)))
  "Returns full string definition for message of type '<mode_msg>"
  (cl:format cl:nil "string mode~%string mission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mode_msg)))
  "Returns full string definition for message of type 'mode_msg"
  (cl:format cl:nil "string mode~%string mission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mode_msg>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'mode))
     4 (cl:length (cl:slot-value msg 'mission))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mode_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'mode_msg
    (cl:cons ':mode (mode msg))
    (cl:cons ':mission (mission msg))
))
