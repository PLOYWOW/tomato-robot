; Auto-generated. Do not edit!


(cl:in-package handeye_cam-msg)


;//! \htmlinclude Msg.msg.html

(cl:defclass <Msg> (roslisp-msg-protocol:ros-message)
  ((w
    :reader w
    :initarg :w
    :type cl:integer
    :initform 0)
   (h
    :reader h
    :initarg :h
    :type cl:integer
    :initform 0)
   (num
    :reader num
    :initarg :num
    :type cl:integer
    :initform 0))
)

(cl:defclass Msg (<Msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name handeye_cam-msg:<Msg> is deprecated: use handeye_cam-msg:Msg instead.")))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handeye_cam-msg:w-val is deprecated.  Use handeye_cam-msg:w instead.")
  (w m))

(cl:ensure-generic-function 'h-val :lambda-list '(m))
(cl:defmethod h-val ((m <Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handeye_cam-msg:h-val is deprecated.  Use handeye_cam-msg:h instead.")
  (h m))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handeye_cam-msg:num-val is deprecated.  Use handeye_cam-msg:num instead.")
  (num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Msg>) ostream)
  "Serializes a message object of type '<Msg>"
  (cl:let* ((signed (cl:slot-value msg 'w)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'h)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Msg>) istream)
  "Deserializes a message object of type '<Msg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'w) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'h) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Msg>)))
  "Returns string type for a message object of type '<Msg>"
  "handeye_cam/Msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Msg)))
  "Returns string type for a message object of type 'Msg"
  "handeye_cam/Msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Msg>)))
  "Returns md5sum for a message object of type '<Msg>"
  "e7443ea74a07cdd552dd4948fd2619cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Msg)))
  "Returns md5sum for a message object of type 'Msg"
  "e7443ea74a07cdd552dd4948fd2619cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Msg>)))
  "Returns full string definition for message of type '<Msg>"
  (cl:format cl:nil "int32 w~%int32 h~%int32 num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Msg)))
  "Returns full string definition for message of type 'Msg"
  (cl:format cl:nil "int32 w~%int32 h~%int32 num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Msg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Msg>))
  "Converts a ROS message object to a list"
  (cl:list 'Msg
    (cl:cons ':w (w msg))
    (cl:cons ':h (h msg))
    (cl:cons ':num (num msg))
))
