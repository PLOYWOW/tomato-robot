; Auto-generated. Do not edit!


(cl:in-package tomato_detection-srv)


;//! \htmlinclude SelectTomato-request.msg.html

(cl:defclass <SelectTomato-request> (roslisp-msg-protocol:ros-message)
  ((gripperPos
    :reader gripperPos
    :initarg :gripperPos
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SelectTomato-request (<SelectTomato-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelectTomato-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelectTomato-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tomato_detection-srv:<SelectTomato-request> is deprecated: use tomato_detection-srv:SelectTomato-request instead.")))

(cl:ensure-generic-function 'gripperPos-val :lambda-list '(m))
(cl:defmethod gripperPos-val ((m <SelectTomato-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomato_detection-srv:gripperPos-val is deprecated.  Use tomato_detection-srv:gripperPos instead.")
  (gripperPos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelectTomato-request>) ostream)
  "Serializes a message object of type '<SelectTomato-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gripperPos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'gripperPos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelectTomato-request>) istream)
  "Deserializes a message object of type '<SelectTomato-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gripperPos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gripperPos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelectTomato-request>)))
  "Returns string type for a service object of type '<SelectTomato-request>"
  "tomato_detection/SelectTomatoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectTomato-request)))
  "Returns string type for a service object of type 'SelectTomato-request"
  "tomato_detection/SelectTomatoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelectTomato-request>)))
  "Returns md5sum for a message object of type '<SelectTomato-request>"
  "740697c9bd3592ba5c586821d8c52d19")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelectTomato-request)))
  "Returns md5sum for a message object of type 'SelectTomato-request"
  "740697c9bd3592ba5c586821d8c52d19")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelectTomato-request>)))
  "Returns full string definition for message of type '<SelectTomato-request>"
  (cl:format cl:nil "float32[] gripperPos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelectTomato-request)))
  "Returns full string definition for message of type 'SelectTomato-request"
  (cl:format cl:nil "float32[] gripperPos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelectTomato-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gripperPos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelectTomato-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SelectTomato-request
    (cl:cons ':gripperPos (gripperPos msg))
))
;//! \htmlinclude SelectTomato-response.msg.html

(cl:defclass <SelectTomato-response> (roslisp-msg-protocol:ros-message)
  ((tomatoPos
    :reader tomatoPos
    :initarg :tomatoPos
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SelectTomato-response (<SelectTomato-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelectTomato-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelectTomato-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tomato_detection-srv:<SelectTomato-response> is deprecated: use tomato_detection-srv:SelectTomato-response instead.")))

(cl:ensure-generic-function 'tomatoPos-val :lambda-list '(m))
(cl:defmethod tomatoPos-val ((m <SelectTomato-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomato_detection-srv:tomatoPos-val is deprecated.  Use tomato_detection-srv:tomatoPos instead.")
  (tomatoPos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelectTomato-response>) ostream)
  "Serializes a message object of type '<SelectTomato-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tomatoPos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'tomatoPos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelectTomato-response>) istream)
  "Deserializes a message object of type '<SelectTomato-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tomatoPos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tomatoPos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelectTomato-response>)))
  "Returns string type for a service object of type '<SelectTomato-response>"
  "tomato_detection/SelectTomatoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectTomato-response)))
  "Returns string type for a service object of type 'SelectTomato-response"
  "tomato_detection/SelectTomatoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelectTomato-response>)))
  "Returns md5sum for a message object of type '<SelectTomato-response>"
  "740697c9bd3592ba5c586821d8c52d19")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelectTomato-response)))
  "Returns md5sum for a message object of type 'SelectTomato-response"
  "740697c9bd3592ba5c586821d8c52d19")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelectTomato-response>)))
  "Returns full string definition for message of type '<SelectTomato-response>"
  (cl:format cl:nil "float32[] tomatoPos~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelectTomato-response)))
  "Returns full string definition for message of type 'SelectTomato-response"
  (cl:format cl:nil "float32[] tomatoPos~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelectTomato-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tomatoPos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelectTomato-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SelectTomato-response
    (cl:cons ':tomatoPos (tomatoPos msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SelectTomato)))
  'SelectTomato-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SelectTomato)))
  'SelectTomato-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectTomato)))
  "Returns string type for a service object of type '<SelectTomato>"
  "tomato_detection/SelectTomato")