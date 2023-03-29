
(cl:in-package :asdf)

(defsystem "custom_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "GetObject" :depends-on ("_package_GetObject"))
    (:file "_package_GetObject" :depends-on ("_package"))
  ))