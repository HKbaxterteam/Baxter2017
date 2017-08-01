
(cl:in-package :asdf)

(defsystem "baxter_gui-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "send_command" :depends-on ("_package_send_command"))
    (:file "_package_send_command" :depends-on ("_package"))
  ))