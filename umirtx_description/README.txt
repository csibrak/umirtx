#readme umirtx_description
src:
parser, ellenőrző, hogy megfelelő-e a szintaktikája az adott URDF-nek.

meshalap: (ezekből a 3D modellekből lettek felületmodellek)
.fcstd freecad project állomány
.stp file-ok freecadból exportált 3D modellek, amik egybevágnak az UMIRTX egyes vázelemeivel.
.dae   példaként benthagyott 3D felület hokuyo robotról (githubról)

meshes: robot egymáshoz képest elmozdulni képes vázelemeinek 3D felületmodelljei
.stl 3D felületmodell állományok, 10mm pontossággal és kritikus részeken 10mm ráhagyással készültek, hogy az ütközésdetektálásnál és a megjelenítésnél tudjam használni 

urdf:  Universal Robot Description Format, robot kinematikai és dinamikai leírására szolgál.
UMI_RTX_URDF.urdf:
	-Denavit-Hartenberg-szerű definiciók sok extrával. 
	-xml formátum, gyakorlatilag bármivel kiegészíthető 
	-fontosabb tartalom: robot neve, csuklók nevei, kapcsolódási pontok és típusok, határértékek, merev vázelemek kapcsolódási pontjai és inerciája, színe, 3D modelljei mind vizualizáláshoz, mind ütközésvizsgálathoz.
	-Az inercia értékek véletlenszerűen vannak kitöltve, hogy szimulációnál ne legyen nullával való osztás.
	

