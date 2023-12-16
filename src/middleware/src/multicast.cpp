#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/if.h>

#include <ros/ros.h>

#define MULTICAST_GROUP "224.16.32.16"
#define MULTICAST_PORT 2918

struct sockaddr_in localSock;
struct ip_mreq group;
int sd;
int datalen;
char databuf[1024];

void init_multicast();
void recv_multicast();

int main(int argc, char *argv[])
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "multicast");
    ros::NodeHandle nh;

    // Loop on 50 hz
    ros::Rate loop_rate(50);

    init_multicast();

    while (ros::ok())
    {
        recv_multicast();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void recv_multicast()
{
    /* Read from the socket. */
    datalen = sizeof(databuf);
    if (read(sd, databuf, datalen) < 0)
    {
        perror("Reading datagram message error");
        close(sd);
        exit(1);
    }
    else
    {
        printf("Reading datagram message...OK.\n");
        printf("The message from multicast server is: \"%s\"\n", databuf);
    }
}

void init_multicast()
{
    /* Create a datagram socket on which to receive. */
    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0)
    {
        perror("Opening datagram socket error");
        exit(1);
    }
    else
        printf("Opening datagram socket....OK.\n");

    /* Enable SO_REUSEADDR to allow multiple instances of this */
    /* application to receive copies of the multicast datagrams. */
    {
        int reuse = 1;
        if (setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse, sizeof(reuse)) < 0)
        {
            perror("Setting SO_REUSEADDR error");
            close(sd);
            exit(1);
        }
        else
            printf("Setting SO_REUSEADDR...OK.\n");
    }

    /* Bind to the proper port number with the IP address */
    /* specified as INADDR_ANY. */
    memset((char *)&localSock, 0, sizeof(localSock));
    localSock.sin_family = AF_INET;
    localSock.sin_port = htons(MULTICAST_PORT);
    localSock.sin_addr.s_addr = INADDR_ANY;
    if (bind(sd, (struct sockaddr *)&localSock, sizeof(localSock)))
    {
        perror("Binding datagram socket error");
        close(sd);
        exit(1);
    }
    else
        printf("Binding datagram socket...OK.\n");

    /* Join the multicast group
     *
     * Note that this IP_ADD_MEMBERSHIP option must be called for each local interface
     * over which the multicast datagrams are to be received.
     */
    group.imr_multiaddr.s_addr = inet_addr(MULTICAST_GROUP);
    group.imr_interface.s_addr = INADDR_ANY;
    if (setsockopt(sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group)) < 0)
    {
        perror("Adding multicast group error");
        close(sd);
        exit(1);
    }
    else
        printf("Adding multicast group...OK.\n");
}