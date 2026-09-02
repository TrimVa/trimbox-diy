# TrimBox DIY

Enregistreur de télémétrie GPS/inertiel pour voitures RC, bâti sur un
**Seeed XIAO nRF52840 Sense**.

L'appareil enregistre seul, sans téléphone, sur sa mémoire flash interne de
2 Mo. Une console web se connecte ensuite en Bluetooth pour piloter
l'enregistrement, récupérer les sessions et les analyser : chronométrage au
tour, trajectoire sur fond satellite, profils de vitesse, forces G.

La compilation se fait **en ligne**, sur les serveurs de GitHub : aucun PC
n'est nécessaire pour produire un firmware et le flasher depuis un téléphone.

---

## Contenu du dépôt

```
trimbox_diy/
    trimbox_diy.ino                 ← firmware
trimbox-diy-console.html            ← console Bluetooth (usage réel)
trimbox-diy-console-demo.html       ← démonstration, sans matériel
.github/workflows/build.yml         ← compilation automatique
```

Le dossier du croquis doit porter **exactement le même nom** que le fichier
`.ino` qu'il contient : c'est une exigence d'Arduino.

⚠️ **Un seul fichier `.ino` dans tout le dépôt.** S'il en reste un ancien, la
compilation s'arrête avec un message explicite plutôt que de risquer de
produire le mauvais firmware.

---

## Matériel

| Élément | Détail |
|---|---|
| Carte | Seeed XIAO nRF52840 **Sense** (variante non-mbed) |
| Mémoire | P25Q16H, 2 Mo QSPI — environ 25 800 points |
| Centrale inertielle | LSM6DS3, accéléromètre réglé sur **±16 g** |
| GNSS | Récepteur u-blox sur `Serial1`, 25 Hz |
| Alimentation | Batterie LiPo, gestion de charge intégrée |

Autonomie mémoire : environ **17 min à 25 Hz**, 43 min à 10 Hz, 86 min à 5 Hz,
7 h à 1 Hz. Le filtre « suspendre à l'arrêt » économise beaucoup en usage
piste, puisque rien n'est stocké au stand.

---

## Personnaliser un module

En tête de `trimbox_diy.ino` :

```c
#define DEVICE_NICKNAME "Buggy 1"
```

Le module s'annonce alors sous le nom Bluetooth `TrimBox Buggy 1`, ce qui
permet de distinguer plusieurs appareils allumés en même temps. Le pseudo
s'affiche dans la console, au démarrage et dans le rapport série.

Restez **sous 16 caractères** : au-delà, la trame d'annonce Bluetooth déborde
et le nom est tronqué.

---

## Compiler depuis un téléphone

1. Onglet **Actions** du dépôt.
2. **Compilation du firmware** → **Run workflow**.
3. Attendre 1 à 4 min (la première compilation est la plus longue ; le core
   est ensuite mis en cache).
4. Ouvrir la compilation terminée, section **Artifacts** en bas de page.

| Artefact | Contenu |
|---|---|
| **firmware-uf2** | Le `.uf2` seul, **sans archive à décompresser** |
| **firmware-complet** | `.uf2`, `.zip` DFU et `.hex`, en archive |

Il faut être **connecté à GitHub** pour télécharger un artefact, même sur un
dépôt public. C'est la cause d'échec la plus fréquente sur mobile.

Modifier le code depuis un téléphone se fait dans l'interface web de GitHub :
ouvrir le `.ino`, bouton crayon, valider. La compilation démarre seule.

---

## Flasher

### Par câble USB — le plus fiable

Nécessite un adaptateur **USB-C OTG** (un câble C-vers-C suffit souvent).

1. Brancher la carte.
2. **Double appui rapide sur RESET** (moins d'une demi-seconde entre les deux).
3. Un lecteur amovible `XIAO-SENSE` apparaît.
4. Y copier le `.uf2`. La carte redémarre seule.

ℹ️ Une **erreur de déconnexion** apparaît souvent en fin de copie : elle est
normale. La carte redémarre dès le dernier bloc écrit, avant que le système
ait terminé sa comptabilité de fichiers. Pour vérifier que le bon firmware
tourne, comparer l'empreinte de compilation affichée par la console à l'heure
de la compilation GitHub.

### Par Bluetooth — sans câble

Le bootloader gère la mise à jour par les airs, à l'aide du `.zip` DFU et de
l'application nRF Connect. Cela suppose d'avoir basculé la carte en mode OTA,
ce que le firmware ne sait pas encore faire.

---

## Publier une version

Poser un tag attache les fichiers à une release, dont l'URL est stable :

```
git tag v1.0 && git push origin v1.0
```

Depuis un téléphone : onglet **Releases** → *Draft a new release* → créer le
tag → publier.

---

## Commandes de secours (moniteur série, 115200 bauds)

| Touche | Effet |
|---|---|
| `s` | Arrêt d'urgence de l'enregistrement |
| `i` | État complet : mémoire, enregistrement, configuration |
| `z` | Configuration par défaut (les données sont conservées) |
| `?` | Rappel des commandes |

---

## Sécurité en cas de coupure d'alimentation

Le firmware supporte l'arrachement de l'alimentation en cours
d'enregistrement :

- la configuration est écrite **en double exemplaire**, en alternance, avec
  numéro de séquence et somme de contrôle : une coupure pendant une
  sauvegarde ne peut pas détruire la seule copie ;
- l'enregistrement **ne reprend jamais tout seul** après une coupure — la
  carte redémarre accessible, GPS éteint, Bluetooth à pleine puissance ;
- les données déjà enregistrées restent intégralement téléchargeables ;
- les enregistrements incomplets sont écartés à la lecture, pour ne pas
  produire de positions ni de vitesses absurdes.

---

## Notes de compilation

Le workflow n'installe que `SparkFun u-blox GNSS` et `Seeed LSM6DS3`, plus
**adafruit-nrfutil**.

Adafruit TinyUSB, SdFat et Adafruit SPIFlash sont **déjà fournis par le core
Seeed**. Les installer en plus provoque une ambiguïté sur le type `File` entre
`Adafruit_LittleFS` et `SdFat`, qui casse la compilation de `bonding.cpp` dans
Bluefruit. Ce piège ne peut pas se produire en ligne ; si vous compilez aussi
en local et rencontrez cette erreur, déplacez ces trois dossiers **hors de**
`Documents/Arduino/libraries/`.

`adafruit-nrfutil` est appelé par le core en fin de compilation pour fabriquer
le paquet DFU, mais le core ne le fournit pas : sans lui, la compilation
échoue *après* avoir produit le `.hex`.

Le core est figé à `Seeeduino:nrf52@1.1.13` pour que la compilation en ligne
produise le même binaire qu'en local. Pour en changer, modifier la variable
`CORE` dans `.github/workflows/build.yml` **et** incrémenter le suffixe de la
clé de cache.

---

## Console web

Les deux fichiers HTML sont autonomes : aucune dépendance externe, aucune
installation. Le fond satellite est la seule fonction qui demande une
connexion.

Le Bluetooth Web exige une **origine sécurisée** : hébergez
`trimbox-diy-console.html` via GitHub Pages (*Settings → Pages → branche
`main`*) et ajoutez la page à l'écran d'accueil depuis Chrome. Elle fonctionne
ensuite hors ligne.

`trimbox-diy-console-demo.html` simule un appareil complet — données en
direct, mémoire, sessions, tracé — et s'ouvre dans n'importe quel navigateur,
y compris sur un ordinateur sans Bluetooth.

### Fonctions

- Télémétrie en direct à 25 Hz
- Démarrage et arrêt de l'enregistrement, cadence et filtres
- Téléchargement et effacement de la mémoire
- Export **VBO** (RaceChrono Pro, Circuit Tools), **CSV** et **GPX**
- Import de fichiers déjà exportés, pour analyse ou comparaison
- Tracé coloré par vitesse, fond satellite, zoom et déplacement
- Ligne d'arrivée et chronométrage au tour
- Statistiques de passage au survol, tours confondus
- Comparaison de plusieurs sessions, profils et trajectoires superposés

### Format de trame

Le format de trame dérive de la documentation du protocole BLE RaceBox
(révision 8) et est conservé tel quel, car la console s'appuie dessus. Aucune
compatibilité avec l'application RaceBox officielle n'est recherchée ni
maintenue : le nom, le modèle et le numéro de série sont libres.
