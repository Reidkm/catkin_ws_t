#socket_client
socket_client is a test program in order to test 


#build 

'''
$ cd socket_client
$ g++ -o socket_client  ./socket_client.cpp ./TCPClient.cpp ./test_m5_openni2_class.cpp -std=c++11  -I./include/ -L./lib/  `pkg-config --cflags --libs opencv` -lOpenNI2



'''

#execute

'''
$ ./socket_client 127.0.0.1 5050

'''
