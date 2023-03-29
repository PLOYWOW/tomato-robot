; Auto-generated. Do not edit!


(cl:in-package custom_msgs-srv)


;//! \htmlinclude GetObject-request.msg.html

(cl:defclass <GetObject-request> (roslisp-msg-protocol:ros-message)
  ((item
    :reader item
    :initarg :item
    :type cl:string
    :initform "")
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass GetObject-request (<GetObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-srv:<GetObject-request> is deprecated: use custom_msgs-srv:GetObject-request instead.")))

(cl:ensure-generic-function 'item-val :lambda-list '(m))
(cl:defmethod item-val ((m <GetObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:item-val is deprecated.  Use custom_msgs-srv:item instead.")
  (item m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <GetObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:id-val is deprecated.  Use custom_msgs-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetObject-request>) ostream)
  "Serializes a message object of type '<GetObject-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'item))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'item))
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetObject-request>) istream)
  "Deserializes a message object of type '<GetObject-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'item) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'item) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetObject-request>)))
  "Returns string type for a service object of type '<GetObject-request>"
  "custom_msgs/GetObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObject-request)))
  "Returns string type for a service object of type 'GetObject-request"
  "custom_msgs/GetObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetObject-request>)))
  "Returns md5sum for a message object of type '<GetObject-request>"
  "e494291815697d37bb402083d19502dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetObject-request)))
  "Returns md5sum for a message object of type 'GetObject-request"
  "e494291815697d37bb402083d19502dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetObject-request>)))
  "Returns full string definition for message of type '<GetObject-request>"
  (cl:format cl:nil "string item~%int64 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetObject-request)))
  "Returns full string definition for message of type 'GetObject-request"
  (cl:format cl:nil "string item~%int64 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetObject-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'item))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetObject-request
    (cl:cons ':item (item msg))
    (cl:cons ':id (id msg))
))
;//! \htmlinclude GetObject-response.msg.html

(cl:defclass <GetObject-response> (roslisp-msg-protocol:ros-message)
  ((pos_width
    :reader pos_width
    :initarg :pos_width
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GetObject-response (<GetObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-srv:<GetObject-response> is deprecated: use custom_msgs-srv:GetObject-response instead.")))

(cl:ensure-generic-function 'pos_width-val :lambda-list '(m))
(cl:defmethod pos_width-val ((m <GetObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:pos_width-val is deprecated.  Use custom_msgs-srv:pos_width instead.")
  (pos_width m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetObject-response>) ostream)
  "Serializes a message object of type '<GetObject-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'pos_width))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetObject-response>) istream)
  "Deserializes a message object of type '<GetObject-response>"
  (cl:setf (cl:slot-value msg 'pos_width) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'pos_width)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetObject-response>)))
  "Returns string type for a service object of type '<GetObject-response>"
  "custom_msgs/GetObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObject-response)))
  "Returns string type for a service object of type 'GetObject-response"
  "custom_msgs/GetObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetObject-response>)))
  "Returns md5sum for a message object of type '<GetObject-response>"
  "e494291815697d37bb402083d19502dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetObject-response)))
  "Returns md5sum for a message object of type 'GetObject-response"
  "e494291815697d37bb402083d19502dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetObject-response>)))
  "Returns full string definition for message of type '<GetObject-response>"
  (cl:format cl:nil "float64[4] pos_width #[x,y,z,grasp_width]~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetObject-response)))
  "Returns full string definition for message of type 'GetObject-response"
  (cl:format cl:nil "float64[4] pos_width #[x,y,z,grasp_width]~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetObject-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos_width) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetObject-response
    (cl:cons ':pos_width (pos_width msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetObject)))
  'GetObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetObject)))
  'GetObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObject)))
  "Returns string type for a service object of type '<GetObject>"
  "custom_msgs/GetObject")