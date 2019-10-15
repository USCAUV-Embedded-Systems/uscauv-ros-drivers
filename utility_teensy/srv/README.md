Hey all!
So in order to declare a new service you need to make a new service file.

You will find an example service file called Sample.srv. It contains the
format for service files. This service will take in two integers (a and b),
and return an int (result). Your file name should be the name of your
service exactly.

If you add any services, you need to add them to the CMakeLists.txt under
add_service_files and make sure that is uncommented.

More info on services: http://wiki.ros.org/srv
