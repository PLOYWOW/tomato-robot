;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::GetObject)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'GetObject (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::GETOBJECT")
  (make-package "CUSTOM_MSGS::GETOBJECT"))
(unless (find-package "CUSTOM_MSGS::GETOBJECTREQUEST")
  (make-package "CUSTOM_MSGS::GETOBJECTREQUEST"))
(unless (find-package "CUSTOM_MSGS::GETOBJECTRESPONSE")
  (make-package "CUSTOM_MSGS::GETOBJECTRESPONSE"))

(in-package "ROS")





(defclass custom_msgs::GetObjectRequest
  :super ros::object
  :slots (_item _id ))

(defmethod custom_msgs::GetObjectRequest
  (:init
   (&key
    ((:item __item) "")
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _item (string __item))
   (setq _id (round __id))
   self)
  (:item
   (&optional __item)
   (if __item (setq _item __item)) _item)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; string _item
    4 (length _item)
    ;; int64 _id
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _item
       (write-long (length _item) s) (princ _item s)
     ;; int64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _item
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _item (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _id
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass custom_msgs::GetObjectResponse
  :super ros::object
  :slots (_pos_width ))

(defmethod custom_msgs::GetObjectResponse
  (:init
   (&key
    ((:pos_width __pos_width) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _pos_width __pos_width)
   self)
  (:pos_width
   (&optional __pos_width)
   (if __pos_width (setq _pos_width __pos_width)) _pos_width)
  (:serialization-length
   ()
   (+
    ;; float64[4] _pos_width
    (* 8    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[4] _pos_width
     (dotimes (i 4)
       (sys::poke (elt _pos_width i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[4] _pos_width
   (dotimes (i (length _pos_width))
     (setf (elt _pos_width i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass custom_msgs::GetObject
  :super ros::object
  :slots ())

(setf (get custom_msgs::GetObject :md5sum-) "e494291815697d37bb402083d19502dd")
(setf (get custom_msgs::GetObject :datatype-) "custom_msgs/GetObject")
(setf (get custom_msgs::GetObject :request) custom_msgs::GetObjectRequest)
(setf (get custom_msgs::GetObject :response) custom_msgs::GetObjectResponse)

(defmethod custom_msgs::GetObjectRequest
  (:response () (instance custom_msgs::GetObjectResponse :init)))

(setf (get custom_msgs::GetObjectRequest :md5sum-) "e494291815697d37bb402083d19502dd")
(setf (get custom_msgs::GetObjectRequest :datatype-) "custom_msgs/GetObjectRequest")
(setf (get custom_msgs::GetObjectRequest :definition-)
      "string item
int64 id
---
float64[4] pos_width #[x,y,z,grasp_width]
")

(setf (get custom_msgs::GetObjectResponse :md5sum-) "e494291815697d37bb402083d19502dd")
(setf (get custom_msgs::GetObjectResponse :datatype-) "custom_msgs/GetObjectResponse")
(setf (get custom_msgs::GetObjectResponse :definition-)
      "string item
int64 id
---
float64[4] pos_width #[x,y,z,grasp_width]
")



(provide :custom_msgs/GetObject "e494291815697d37bb402083d19502dd")


