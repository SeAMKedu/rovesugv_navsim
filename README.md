![hankelogo](/images/rovesugv_logo.png)

# RovesUGV - Navsim

ROS 2 -paketti nelipyöräisen mobiilirobotin GPS-navigoinnin simulointiin.

## Ohjelmistoriippuvuudet

* ROS 2 Humble ([asennus](https://docs.ros.org/en/humble/Installation.html))
* Ignition Gazebo Fortress ([asennus](https://gazebosim.org/docs/fortress/install/))
* Joukko ROS 2 -paketteja, jotka voidaan asentaa alla olevilla komennoilla 

```
source /opt/ros/humble/setup.bash

sudo apt install ros-humble-ros-gz
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-mapviz
sudo apt install ros-humble-mapviz-plugins
sudo apt install ros-humble-tile-map
sudo apt install ros-teleop-twist-keyboard
```

* customtkinter GUI-sovellusta varten
```
pip3 install customtkinter
```

* Docker
* ROS Offline Google Maps for MapViz ([asennus](https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite))

## Simulaation ajaminen

1. Käynnistä MapProxy-palvelin (Huomautus: tämä täytyy tehdä ainoastaan kerran tietokoneen käynnistämisen jälkeen).
```
sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
```
2. Avaa terminaali ja mene omaan ROS 2 workspace -kansioosi
```
cd path/to/my/ros2/workspace/folder
```
3. Lataa ympäristömuuttujat
```
source install/setup.bash
```
4. Käynnistä simulaatio
```
ros2 launch rovesugv_navsim simulation.launch.py
```
5. Avaa uusi terminaali ROS 2 workspace -kansiossa ja lataa tarvittavat ympäristömuuttujat. Käynnistä sitten ROS 2 -solmu, joka vastaanottaa MapViz-kartassa klikatun pisteen GPS-koordinaatit ja navigoi mobiilirobotin kyseiseen pisteeseen
```
source install/setup.bash
ros2 run rovesugv_navsim gps_waypoint_commander
```
6. Vaihtoehtoisesti voit käynnistää uudessa terminaalissa GUI-sovelluksen, jolla voi ohjata mobiilirobottia ennalta määrättyihin GPS-koordinaatteihin
```
source install/setup.bash
ros2 run rovesugv_navsim gui_waypoint_commander
```

Simulaation voi sulkea painamalla Ctrl+c.

## Gazebo

Mobiilirobotin ja sen antureiden simulointi. Simulointiympäristöön voidaan lisätä kohteita (esim. kuutio tai sylinteri) vasemman yläkulman valikosta.

![gazebosim](/images/gazebosim.png)

## RViz2

Anturidatan (esim. lidar) visualisointi.

![rviz](/images/rviz2.png)

## MapViz

MapViz-sovellus näyttää mobiilirobotin sijainnin karttapohjalla. Jos "gps_waypoint_commander"-solmu on käynnissä, kartalla voi klikata pistettä, johon haluaa mobiilirobotin liikkuvan.

![mapviz](/images/mapviz.png)

## GUI-sovellus

Python-sovellus, jolla voi ajaa mobiilirobottia ennalta määritettyihin GPS-koordinaattipisteisiin, jotka on asetettu "config"-kansiossa olevassa "gps_waypoints.yaml"-tiedostossa.

![gui_app](/images/gui_app.png)

Sovelluksen voi sulkea oikean yläkulman rastista.

## Tekijätiedot

Hannu Hakalahti, Asiantuntija, TKI, Seinäjoen ammattikorkeakoulu (SEAMK).

## RovesUGV-hanke

RovesUGV-hanke keskittyy autonomisten logistiikkaratkaisujen kehittämiseen ja demonstrointiin Roveksen teollisuusalueella. Hankkeen tarpeen taustalla on Roveksen ja Kapernaumin teollisuusalueiden yritysten välinen jatkuva logistiikka ja tavaraliikenne, joka nykyisin toimii yritysten oman työvoiman, pakettiautojen, ja isompien kuorma-autojen avulla. Hankkeen tavoitteena on kehittää Proof-of-Concept (PoC) demo, jossa tavaraa siirretään autonomisesti Husarion Panther UGV -mobiilirobotin avulla.

* Hankkeen nimi: RovesUGV
* Hankkeen aikataulu: 01.04.2025 - 31.07.2026
* Hankkeen rahoittaja: Etelä-Pohjanmaan liitto, Euroopan aluekehitysrahasto (EAKR)

---
![eakr_logo](/images/Euroopan_unionin_osarahoittama_POS.png)

![epliitto_logo](/images/EPLiitto_logo_vaaka_vari.jpg)

![seamk_logo](/images/SEAMK_vaaka_fi_en_RGB_1200x486.jpg)