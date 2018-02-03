#readme umirtx_hardware package
src:   (include mappa is ide tartozik a megfelelő fejlécekkel)
umirtx_base.cpp:  Alap definiciók a működéshez.
control thread felelős az egyes infók továbbításáért a különböző programrészek között egy külön szálon.
main():
kapcsolódást kezdeményez a "robot" hoz azaz összekapcsolja a controllereket és a saját programrészeket.
fenntartja magát, "parent" ként a rosspin()-nel. 

umirtx_hardware.cpp: konkrét UDP socket kezelés, adatstruktúra definiciókat és állandókat tartalmaz, valamint adatok átalakítására szolgál, hogy a motorvezérlők fel tudják dolgozni a parancsokat és vissz-irányban is ez történik.
névtér definiálása.
Osztály umirtx_node létrehozása
UDP client definiciója 
	-új socket, ha nincs még
	-intre alakítás floatról+ a differenciálmű lekezelése.
	-adatküldés a motorvezérlőre
	-ha jött adat, akkor (jelenleg ellenőrzés nélkül) átmásolja egy bufferbe.
	-bufferből átalakítás int-ről float-ra és differenciálmű visszaszámolása
	-ha nincs adat lezárja a socketet
Többi függvénynek nincs haszna igazán ebben az implementációban, átalakításnál hasznos lehet ötletnek.

konstruktor definicióks stb.
	Létrehozza/spawnolja a position_joint_interface-t és joint_state_interface-t mint objektumokat.
	ezekkel lehet kapcsolódni a joint_state_publisher és a controllerekhez.



Launcher: Kezdeti paraméterek, argumentumok adása a programoknak.
umirtx_node.launch:
	node név:umirtx_node
	package info:, type..
	argumentumok: 	100Hz a frissítési frekvencia. 192.168.1.200 a motorvezérlő IP címe.
default_controllers.launch: alapértelmezett ("szabványos") vezérlők paraméterezésének betöltése (yaml file-okból) és indítása megfelelő sorrendben.
	controller manager kezeli a vezérlőket, spawnol/példányosít és killez.
	.yaml file-okban sok sok paramétert lehet adni a vezérlőknek, aminek a ROS tutorialokban és githubon lehet utánanézni.
	pl: rosparam command="load" file="xy.yaml"
	figyelem, nagyon érzékenyek a yaml file-ok a szintaktikára, egy space is elrontja a sor elején akár!
config:
controllers.yaml:
	arm és gripper definiciója, hogy milyen vezérlőket kell példányosítania a controller_managernek (szintaktikára figyelni! egy darap space|tab sem lehet benne feleslegesen!)
