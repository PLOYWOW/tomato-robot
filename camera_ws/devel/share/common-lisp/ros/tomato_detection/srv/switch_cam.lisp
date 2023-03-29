; Auto-generated. Do not edit!


(cl:in-package tomato_detection-srv)


;//! \htmlinclude switch_cam-request.msg.html

(cl:defclass <switch_cam-request> (roslisp-msg-protocol:ros-message)
  ((req_status
    :reader req_status
    :initarg :req_status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass switch_cam-request (<switch_cam-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <switch_cam-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'switch_cam-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tomato_detection-srv:<switch_cam-request> is deprecated: use tomato_detection-srv:switch_cam-request instead.")))

(cl:ensure-generic-function 'req_status-val :lambda-list '(m))
(cl:defmethod req_status-val ((m <switch_cam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomato_detection-srv:req_status-val is deprecated.  Use tomato_detection-srv:req_status instead.")
  (req_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <switch_cam-request>) ostream)
  "Serializes a message object of type '<switch_cam-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'req_status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <switch_cam-request>) istream)
  "Deserializes a message object of type '<switch_cam-request>"
    (cl:setf (cl:slot-value msg 'req_status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<switch_cam-request>)))
  "Returns string type for a service object of type '<switch_cam-request>"
  "tomato_detection/switch_camRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'switch_cam-request)))
  "Returns string type for a service object of type 'switch_cam-request"
  "tomato_detection/switch_camRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<switch_cam-request>)))
  "Returns md5sum for a message object of type '<switch_cam-request>"
  "c1d0f6b5d3ccfee714fe1fdff0074c75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'switch_cam-request)))
  "Returns md5sum for a message object of type 'switch_cam-request"
  "c1d0f6b5d3ccfee714fe1fdff0074c75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<switch_cam-request>)))
  "Returns full string definition for message of type '<switch_cam-request>"
  (cl:format cl:nil "bool req_status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'switch_cam-request)))
  "Returns full string definition for message of type 'switch_cam-request"
  (cl:format cl:nil "bool req_status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <switch_cam-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <switch_cam-request>))
  "Converts a ROS message object to a list"
  (cl:list 'switch_cam-request
    (cl:cons ':req_status (req_status msg))
))
;//! \htmlinclude switch_cam-response.msg.html

(cl:defclass <switch_cam-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass switch_cam-response (<switch_cam-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <switch_cam-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'switch_cam-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tomato_detection-srv:<switch_cam-response> is deprecated: use tomato_detection-srv:switch_cam-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <switch_cam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomato_detection-srv:data-val is deprecated.  Use tomato_detection-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <switch_cam-response>) ostream)
  "Serializes a message object of type '<switch_cam-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <switch_cam-response>) istream)
  "Deserializes a message object of type '<switch_cam-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<switch_cam-response>)))
  "Returns string type for a service object of type '<switch_cam-response>"
  "tomato_detection/switch_camResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'switch_cam-response)))
  "Returns string type for a service object of type 'switch_cam-response"
  "tomato_detection/switch_camResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<switch_cam-response>)))
  "Returns md5sum for a message object of type '<switch_cam-response>"
  "c1d0f6b5d3ccfee714fe1fdff0074c75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'switch_cam-response)))
  "Returns md5sum for a message object of type 'switch_cam-response"
  "c1d0f6b5d3ccfee714fe1fdff0074c75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<switch_cam-response>)))
  "Returns full string definition for message of type '<switch_cam-response>"
  (cl:format cl:nil "uint8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'switch_cam-response)))
  "Returns full string definition for message of type 'switch_cam-response"
  (cl:format cl:nil "uint8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <switch_cam-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <switch_cam-response>))
  "Converts a ROS message object to a list"
  (cl:list 'switch_cam-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'switch_cam)))
  'switch_cam-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'switch_cam)))
  'switch_cam-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'switch_cam)))
  "Returns string type for a service object of type '<switch_cam>"
  "tomato_detection/switch_cam")