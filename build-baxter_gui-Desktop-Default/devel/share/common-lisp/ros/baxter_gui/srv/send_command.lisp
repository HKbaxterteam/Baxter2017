; Auto-generated. Do not edit!


(cl:in-package baxter_gui-srv)


;//! \htmlinclude send_command-request.msg.html

(cl:defclass <send_command-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type cl:string
    :initform ""))
)

(cl:defclass send_command-request (<send_command-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <send_command-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'send_command-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name baxter_gui-srv:<send_command-request> is deprecated: use baxter_gui-srv:send_command-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <send_command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader baxter_gui-srv:start-val is deprecated.  Use baxter_gui-srv:start instead.")
  (start m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <send_command-request>) ostream)
  "Serializes a message object of type '<send_command-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'start))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'start))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <send_command-request>) istream)
  "Deserializes a message object of type '<send_command-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'start) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<send_command-request>)))
  "Returns string type for a service object of type '<send_command-request>"
  "baxter_gui/send_commandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'send_command-request)))
  "Returns string type for a service object of type 'send_command-request"
  "baxter_gui/send_commandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<send_command-request>)))
  "Returns md5sum for a message object of type '<send_command-request>"
  "c856a68efb24b22273f35f912e1cb37c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'send_command-request)))
  "Returns md5sum for a message object of type 'send_command-request"
  "c856a68efb24b22273f35f912e1cb37c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<send_command-request>)))
  "Returns full string definition for message of type '<send_command-request>"
  (cl:format cl:nil "string start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'send_command-request)))
  "Returns full string definition for message of type 'send_command-request"
  (cl:format cl:nil "string start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <send_command-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'start))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <send_command-request>))
  "Converts a ROS message object to a list"
  (cl:list 'send_command-request
    (cl:cons ':start (start msg))
))
;//! \htmlinclude send_command-response.msg.html

(cl:defclass <send_command-response> (roslisp-msg-protocol:ros-message)
  ((answ
    :reader answ
    :initarg :answ
    :type cl:string
    :initform ""))
)

(cl:defclass send_command-response (<send_command-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <send_command-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'send_command-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name baxter_gui-srv:<send_command-response> is deprecated: use baxter_gui-srv:send_command-response instead.")))

(cl:ensure-generic-function 'answ-val :lambda-list '(m))
(cl:defmethod answ-val ((m <send_command-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader baxter_gui-srv:answ-val is deprecated.  Use baxter_gui-srv:answ instead.")
  (answ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <send_command-response>) ostream)
  "Serializes a message object of type '<send_command-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'answ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'answ))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <send_command-response>) istream)
  "Deserializes a message object of type '<send_command-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'answ) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'answ) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<send_command-response>)))
  "Returns string type for a service object of type '<send_command-response>"
  "baxter_gui/send_commandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'send_command-response)))
  "Returns string type for a service object of type 'send_command-response"
  "baxter_gui/send_commandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<send_command-response>)))
  "Returns md5sum for a message object of type '<send_command-response>"
  "c856a68efb24b22273f35f912e1cb37c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'send_command-response)))
  "Returns md5sum for a message object of type 'send_command-response"
  "c856a68efb24b22273f35f912e1cb37c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<send_command-response>)))
  "Returns full string definition for message of type '<send_command-response>"
  (cl:format cl:nil "string answ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'send_command-response)))
  "Returns full string definition for message of type 'send_command-response"
  (cl:format cl:nil "string answ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <send_command-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'answ))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <send_command-response>))
  "Converts a ROS message object to a list"
  (cl:list 'send_command-response
    (cl:cons ':answ (answ msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'send_command)))
  'send_command-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'send_command)))
  'send_command-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'send_command)))
  "Returns string type for a service object of type '<send_command>"
  "baxter_gui/send_command")