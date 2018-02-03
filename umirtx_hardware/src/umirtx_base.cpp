#include <string>
#include <cstdlib>
#include <boost/asio/io_service.hpp> //io kezeléshez, ethernetnek függősége lehet ebben az állományban.
#include <boost/thread.hpp> //többszálú program létrehozásához tartalmazza a fejléceket.
#include "controller_manager/controller_manager.h" //R.O.S. controller manager fejléce. 
#include "umirtx_hardware.h" //tartalmazza a szükséges fejlécet
#include "ros/ros.h" //R.O.S. keretrendszer fejléceit tartalmazza
#include <signal.h> //  our new library

/*communication module for UMIRTX robot
 * real umirtx robot <-> umirtx_node <-> ROS(moveit)
*/

volatile sig_atomic_t flag = 0;
void umirtxSigintHandler(int arg);

/* controlThread(
 * Rate rate // meghívások frekvenciája Hz-ben
 * , UmirtxHardware* robot // Robot struktúrájára mutató pointer
 * , ControllerManager* cm // vezérlő manager pointere
 * )
 *
 * Ennek a függvénynek feladata, hogy indítsa a nem blokkolható szálat a megfelelő paraméterekkel, az információcsere koordinálása és kezelje a leállítást.
 *
 *
*/
void controlThread(ros::Rate rate, umirtxbase::UmirtxHardware* robot, controller_manager::ControllerManager* cm/*, boost::shared_ptr<int> * mute*/) //új szálon futó aszinkron control manager
{
  ros::Time last_time; //Ez tartalmazza az utolsó időpontot és nullázni kell használat előtt.
    int controlThreadcounter=0;	//biztonsági számláló és mérési feladatokra használható, nullázás kell.  
  while (flag==0) //folyamatos ismétlődő futás, amíg le nem állítjük kívülről
  {
    ros::Time this_time = ros::Time::now(); //aktuális időbélyeg
   //robot->read(); //olvasás blokkolva
    cm->update(this_time, this_time - last_time); //eltelt idővel frissítjük a control managert
    robot->write();	//elküldjük az új célt a robotnak blokkolva (szinkronizálva a külön szálra.
    last_time = this_time;	//következő számoláshoz elmenti az időbélyeget
    rate.sleep(); //várakozási időt ezzel lehet változtatni. 
   //ROS_INFO("controlthread #%d %d",controlThreadcounter,this_time.nsec); //hibakeresésen kívül nem szabad használni, mert nagyon sok logot generál
    controlThreadcounter++; //biztonsági számláló, azonosítási célt szolgál
    robot->UmirtxUDPclient(robot->umirtx_ip); //UDP-n érkező adatok feldolgozása
    //ROS_INFO("flag: %d",flag);
  }
  ROS_INFO("exit cought: %d",flag);
//   umirtxSigintHandler();
  pthread_exit(NULL); //nem biztos, hogy kell


}
boost::thread t;
int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "umirtxnode");

  //argumentumok ellenőrzése:
  if((atoi(argv[1])<1)||((atoi(argv[1])>2000))) //egyszerű argumentumok számának és értékek ellenőrzése
  {
      ROS_INFO("usage: umirtxnode <rate in Hz,example:(100)> <ip-addr,example:(127.0.0.1)>  %d",atoi(argv[1]) );
      return -1;
  }
  if((strlen(argv[2])>16)&&((strlen(argv[2])<7))){ROS_ERROR("WRONG IP ADDRESS in umirtx_node\r\n");return -1;}
  char umirtx_ip[strlen(argv[2])];
  strcpy(umirtx_ip,argv[2]);
  umirtxbase::UmirtxHardware robot; //osztálypéldányosítása
  signal(SIGINT,umirtxSigintHandler);
  robot.umirtx_ip=umirtx_ip; 	//Beállítás: motorvezérlő IPcím
  ROS_INFO("robot megvan connecting %s with %s Hz rate",argv[1],argv[2]);
  // create UDP stuff 
 // boost::thread(boost::bind(&boost::asio::io_service::run, &io_service)); //többszálú rendszer esetén kellhet
  // Background thread for the controls callback.
  ros::NodeHandle controller_nh("");
  ROS_INFO("controllernh ok");
  controller_manager::ControllerManager cm(&robot, controller_nh); //controller manager példányosítása és paraméteres indítása
  ROS_INFO("cm megvan"); //infó a felhasználó fele
  //boost::shared_ptr <int> mut{new int{1}};
  t=boost::thread(boost::bind(controlThread, ros::Rate(atol(argv[1])), &robot, &cm/*, &mut*/)); // új szál indítása a driverhez
  ROS_INFO("boost megvan");
  // Foreground ROS spinner for ROS callbacks, including ...
  ros::spin(); //folyamatos futtatásra parancs és blokkol is.
ROS_INFO("spin megvan"); //infó a felhasználó fele.
//mut=NULL;
return 0;
}

void umirtxSigintHandler(int arg ){
    ROS_INFO("Sigint ok");
    t.detach();
    ros::shutdown();
}
