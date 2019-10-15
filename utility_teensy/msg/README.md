Here is where you declare your custom messages. Custom messages are good
ways to send packets of information.

Your message file needs to declare the types of data that are sent in your
custom message. You will find a sample messages file called Sample.msg
that declares a message that sends a string, int and float.

If you make any new messages, you need to add them to the CMakeLists.txt
under add_message_files, and uncomment that section.

More info on messages: http://wiki.ros.org/msg
