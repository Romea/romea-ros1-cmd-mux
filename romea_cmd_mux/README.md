# 1 Overview #

This package provides a generic command mutiplexer based on ideas found in topic_tools mux, twist_mux or cmd_vel_mux nodes.  It takes N input topics and outputs the messages from a single one. For selecting the topic they are prioritized based on their priority and their acitivity. Finally, thanks to ros_type_introspection any kind of messages can be mutiplexed.    

### Subscribed Topics ###

-  cmd_input_N (any::any)

    The Nth input topic that mux is subscribed to (specified via connect service)

### 2. Published Topics ###

- cmd_out (any::any)

   Messages from the selected input are published into this topic

### 2. Service servers ###

- connect (romea_cmd_mux::Connect)

  This service is used to try to add an input topic according three informations :  

     -  topic : name of the topic
     -  timeout : messages timeout in seconds. If no message arrives after the timeout, another active topic is selected.
     -  priority : priority of the topic from 0 to 255. The higher the more priority it has over the others.

  Connect request can failed if an input topic is already resgister with the same name or if a input topic has already the same priority.      

- connect (romea_cmd_mux::Disconnect)

  This service is used to remove an input topic.
