/**
 * Based on Clearpath robotics
 *  \file
 *  \brief      Class representing UMIRTX (BM14W4) hardware
 *  \author     Tamas Csibrak <tamas.csibrak@gmail.com>
 *  \copyright  Copyright (c) 2014, Budapest University of Technology and Economics, Department of Manufacturing .
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to tamas.csibrak@gmail.com
 *
 */
/*
#define GRIPPER motor3
#define JOINT1
#define JOINT2
#define JOINT3  4
#define JOINT4  6
#define JOINT5
#define JOINT6  5

#define MOTOR1 JOINT1/2
#define MOTOR2 JOINT7B
#define MOTOR3 JOINT6
#define MOTOR4 JOINT4
#define MOTOR5 JOINT7A
#define MOTOR6 JOINT5
#define MOTOR7 JOINT3
motor[0].pos=0;
motor[1].pos=joints_[1].pos*MOTORMPLY1;
motor[2].pos=joints_[7].pos-joints_[6].pos;
motor[3].pos=joints_[5].pos;
motor[4].pos=joints_[3].pos;
motor[5].pos=joints_[6].pos+joints_[7].pos;
motor[6].pos=joints_[4].pos;
motor[7].pos=joints_[2].pos;
*/
#define MOTORMPLY1 (1.1635e-3*0.25) //magassági
#define MOTORMPLY2 (-2.44e-3*0.25) // joint2 Arm váll
#define MOTORMPLY3 (3.37e-3*0.25) // joint3 Arm könyök
#define MOTORMPLY4 (2.61e-3*0.25) //joint4 Arm forgat
#define MOTORMPLY5 (-2.61e-3*0.25) //joint5 Arm vege fel-le
#define MOTORMPLY6 (-1e-3*0.25) //joint6 Arm vege csavar
#define MOTORMPLY7 (-2.5e-4*0.25) //joint7 gripperA+gripperB

#define JOINT2TO3MPLY 2.08



#include <boost/assign.hpp>
#include "umirtx_hardware.h"
//#include "include/umirtx_hardware/umirtx_hardware.h"

namespace umirtxbase
{

void UmirtxHardware::UmirtxUDPclient(const char * umirtx_ip) //ez a programrész küld és fogad UDP csomagokat
{
    if(1)
    {
        int i;
        for(i=0;i<8;i++){
         joints_[i].position=joints_[i].position_command;
         joints_[i].effort=0.0;
         joints_[i].velocity=0.0;
        }
        if(joints_[0].position<0.2){joints_[0].position=0.2;}
    }
    else{
    static struct sockaddr_in si_other,si_me; //Példakódból deklarációk
    static int s, i;
    socklen_t slen=sizeof(si_other); //
    static char buf[BUFLEN]; //statikus buffer lefoglalása
    if(connect_ok==false) //első indítás ellenőrzése
    {
        connect_ok= true; //következő indítási algoritmus megakadályozására "merker"
        if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) //udp socket létrehozása
        {
            connect_ok=false;ROS_INFO("socket error -1");exit;
        }
        if(setsockopt(s,SOL_SOCKET,SO_RCVTIMEO,&timevalue,sizeof(timevalue))<0)perror("Error timeout");
        memset((char *) &si_other, 0, sizeof(si_other));
        si_other.sin_family = AF_INET;
        si_other.sin_port = htons(PORT);
        if (inet_aton(umirtx_ip, &si_other.sin_addr)==0) {
            fprintf(stderr, "inet_aton() failed\n");
            connect_ok=false;
        }
        ROS_INFO("udp reconfig");
    }
    ros::Time this_time = ros::Time::now();
    int j;
    int32_t k[8];

        k[7]=(int32_t)(joints_[1].position_command*(1/MOTORMPLY1)); //váll
        k[5]=(int32_t)((joints_[3].position_command+joints_[2].position_command/JOINT2TO3MPLY)*(1/MOTORMPLY3)); //csukló
        k[3]=(int32_t)(joints_[2].position_command*(1/MOTORMPLY2)); //könyök
        k[6]=(int32_t)((joints_[4].position_command-joints_[5].position_command)*(1/MOTORMPLY4)); //diff bal
        k[4]=(int32_t)((joints_[5].position_command+joints_[4].position_command)*(1/MOTORMPLY4)); //diff jobb
        k[1]=(int32_t)(joints_[0].position_command*(1/MOTORMPLY6)); //magassági
        k[2]=(int32_t)((joints_[7].position_command-joints_[6].position_command)*(1/MOTORMPLY7));//gripper
        //k[j]=(int32_t)(250*(sin((double)this_time.sec+(double)this_time.nsec/1000000000)));

    //ROS_INFO("csomagmeret %d ",sizeof(k));
    memcpy((void*) buf,(void*)k,sizeof(k));
    //sprintf(buf, "%d,%d\n", i++ , this_time.nsec ); //hibakereséshez
    //memcpy((void*) &buf,(const void*)&packs,sizeof(packet)); //
    if (sendto(s, buf, BUFLEN, 0,(const sockaddr*) &si_other, slen)==(ssize_t)-1){
        close(s);ROS_INFO("socket error -1");
        connect_ok=false;
        exit;
    }
    //diep("sendto()");
    //ROS_INFO("udp sending packet#%d ip:%s",i,umirtx_ip);
    ssize_t recvlen = recvfrom(s,(void*)&buf,BUFLEN-2,0,(sockaddr*) &si_other,&slen);
    if(recvlen>0)
    {
        mempcpy((void*)packs,(void*)buf,sizeof(packs));

        joints_[1].position=packs[7].pos*MOTORMPLY1; //ok, skálázás a robot motorjaihoz 
        joints_[5].position=(packs[4].pos*MOTORMPLY4-packs[6].pos*MOTORMPLY4)/2; //A speciális differenciálműves komplex csuklóhoz skálázás
        joints_[2].position=packs[3].pos*MOTORMPLY2; //ok, skálázás a robot motorjaihoz
        joints_[3].position=packs[5].pos*MOTORMPLY3-joints_[2].position/JOINT2TO3MPLY; //ok A speciális differenciálműves komplex csuklóhoz skálázás
        joints_[6].position=-packs[2].pos*MOTORMPLY7/2; //, skálázás a robot motorjaihoz
        joints_[4].position=packs[6].pos*MOTORMPLY4+joints_[5].position; //ok, skálázás a robot motorjaihoz

       // joints_[0].position=packs[7].pos*MOTORMPLY1;
        joints_[7].position=-joints_[6].position; //ezzel alakítható a két külön csuklóból álló megfogó egy csuklóssá

        umiconverter(&packs[7],&joints_[1]);	//Szofisztikáltabb átalakítás
        umiconverter(&packs[5],&joints_[3]);	//Szofisztikáltabb átalakítás
        umiconverter(&packs[3],&joints_[2]);	//Szofisztikáltabb átalakítás
        umiconverter(&packs[6],&joints_[4]);	//Szofisztikáltabb átalakítás
        umiconverter(&packs[4],&joints_[5]);	//Szofisztikáltabb átalakítás
        umiconverter(&packs[2],&joints_[6]);	//Szofisztikáltabb átalakítás
        umiconverter(&packs[1],&joints_[7]);	//Szofisztikáltabb átalakítás

              if(joints_[0].position_command==0.0){
               joints_[0].position=0.7; //inicializálást szolgál. A megoldó ütközési hibával tér vissza 0.0 érték esetén, ezt kiküszöbölendően került ez bele.
              }
              else{
                    joints_[0].position=joints_[0].position_command; //hibakeresés esetén a magassági tengelyt meg lehet kerülni ezze a kóddal.
              }
              //ROS_INFO("effort:%d, pos:%d",packs[1].eff,packs[1].pos); //hibakereséshez felhasználói üzenet
       // ROS_INFO("cmd-k %d, %d, %d, %d, %d, %d, %d, %d ",packs[0].cmd,packs[1].cmd,packs[2].cmd,packs[3].cmd,packs[4].cmd,packs[5].cmd,packs[6].cmd,packs[7].cmd);	//hibakereséshez felhasználói üzenet
    }
    if(connect_ok==false) //ha nincs kapcsolat, akkor kilép a program
    {
        close(s);
    }
    }
}
void UmirtxHardware::umiconverter(packet* P,Joint* J) //Konvertáláshoz készült, előzetes átalakítás egy definicióból. Skálázási művelet.
{
    //J->position=(P->pos)*1e-3;
    //J->position_command=(P->cmd)*1e-3;
    J->effort=P->eff*0.8620;
    J->velocity=P->vel*1e-10;
}

void UmirtxHardware::pcconverter(packet* P,Joint* J) //konvertáláshoz készült, kerekíti és átalakítja a lebegőpontos számokat 32bites előjeles egészre.
{
    P->cmd=(int32_t)round((J->position_command*1e3));
}

UmirtxHardware::UmirtxHardware()
{
    timevalue.tv_sec=0;
    timevalue.tv_usec=500;
    ros::V_string joint_names = boost::assign::list_of("joint1")("joint2")("joint3")("joint4")("joint5")("joint6")("joint7A")("joint7B");
    connect_ok=false;
    for (unsigned int i = 0; i < joint_names.size(); i++) {
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                                &joints_[i].position,  &joints_[i].velocity, &joints_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle(
                    joint_state_handle, &joints_[i].position_command);
        position_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    //feedback_sub_ = nh_.subscribe("feedback",1,&UmirtxHardware::feedbackCallback,this);

}

/**
 * Populates the internal joint state struct from the most recent Feedback message
 * received from the MCU.
 *
 * Called from the controller thread.
 */
void UmirtxHardware::read()
{
    // MUTEX LOCK
    // ROS_INFO("read megvan");
    boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
    if (feedback_msg_ && feedback_msg_lock)
    {
        ROS_INFO("mutex megvan");
        for (int i = 0; i < 7; i++)
        {
            //  joints_[i].position =(feedback_msg_)->drivers[i ].measured_travel;
            // joints_[i].velocity = feedback_msg_->position;
            //  joints_[i].effort = 0; // TODO(mikepurvis): determine this from amperage data.
        }
        joints_[0].position=0.7;
    }
    // READ FROM UDP STUFF
}

void UmirtxHardware::write() //ezt ki kell szedni!
{
    // MUTEX LOCK

    // UDP package based on position_command
}

void UmirtxHardware::feedbackCallback(const  sensor_msgs::JointState::ConstPtr& msg) //direkt beírás
{
    // Update the feedback message pointer to point to the current message. Block
    // until the control thread is not using the lock.
    boost::mutex::scoped_lock lock(feedback_msg_mutex_);
    feedback_msg_ = msg;
    ROS_INFO("feedcallback megvan");
}


}  // namespace umirtxbase
