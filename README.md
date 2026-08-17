# RaceBox Mini S — émulateur nRF52840

Firmware pour Seeed XIAO nRF52840 Sense émulant un RaceBox Mini S, avec
enregistrement autonome sur la mémoire flash QSPI de 2 Mo, et console web
d'analyse.

La compilation se fait **en ligne**, sur les serveurs de GitHub : aucun PC
n'est nécessaire pour produire un firmware et le flasher depuis un téléphone.

---

## Organisation du dépôt

```
nRF52840_racebox_mini_S_emulator/
    nRF52840_racebox_mini_S_emulator.ino    ← le firmware
racebox-console.html                        ← console de pilotage (Bluetooth)
racebox-console-demo.html                   ← démonstration sans matériel
.github/workflows/build.yml                 ← compilation automatique
```

Le dossier du croquis doit porter **exactement le même nom** que le fichier
`.ino` qu'il contient : c'est une exigence d'Arduino. Le workflow recopie
automatiquement le fichier dans une arborescence conforme si ce n'est pas le
cas, mais autant respecter la convention.

---

## Compiler depuis un téléphone

1. Ouvrez le dépôt sur GitHub, onglet **Actions**.
2. Sélectionnez **Compilation du firmware**, puis **Run workflow**.
3. Attendez 1 à 4 minutes (la première compilation est la plus longue :
   le core est ensuite mis en cache).
4. Ouvrez la compilation terminée et téléchargez l'archive **firmware**.

Elle contient trois fichiers :

| Fichier | À quoi il sert |
|---|---|
| `….uf2` | Glisser-déposer sur le lecteur USB de la carte |
| `….-dfu.zip` | Mise à jour par Bluetooth, via nRF Connect |
| `….hex` | Programmation par sonde SWD |

Modifier le code depuis un téléphone se fait directement dans l'interface web
de GitHub : ouvrez le `.ino`, bouton crayon, validez. La compilation part
toute seule.

---

## Flasher depuis un téléphone

### Par câble — le plus fiable

Il faut un adaptateur **USB-C OTG** (un câble C‑vers‑C suffit souvent).

1. Branchez la carte au téléphone.
2. **Double appui rapide sur RESET** (moins d'une demi-seconde entre les deux).
3. Un lecteur amovible `XIAO-SENSE` apparaît dans l'application Fichiers.
4. Copiez-y le `.uf2`. La carte redémarre seule sur le nouveau firmware.

### Sans câble — par Bluetooth

Nécessite d'avoir mis la carte en mode OTA au préalable (voir plus bas ;
cette bascule n'est pas encore implémentée dans le firmware).

1. Application **nRF Connect** → onglet DFU.
2. Sélectionnez le `.zip`, choisissez l'appareil `AdaDFU`.
3. Activez **Force scanning** : au passage en mode OTA, la carte redémarre et
   réapparaît sous un autre nom et une autre adresse.

⚠️ Une mise à jour par Bluetooth réalisée pendant que la carte est branchée à
un ordinateur s'installe correctement mais ne démarre pas sur la nouvelle
version. Débranchez avant.

---

## Publier une version

Poser un tag attache les fichiers à une release, dont l'URL est stable et
directement téléchargeable :

```
git tag v1.0 && git push origin v1.0
```

Depuis un téléphone : onglet **Releases** → *Draft a new release* → créer le
tag → publier. Le workflow y déposera les binaires.

---

## Note sur les bibliothèques

Le workflow n'installe **que** `SparkFun u-blox GNSS` et `Seeed LSM6DS3`.

Adafruit TinyUSB, SdFat et Adafruit SPIFlash sont déjà fournis par le core
Seeed. Les installer en plus provoque une ambiguïté sur le type `File` entre
`Adafruit_LittleFS` et `SdFat`, qui casse la compilation de `bonding.cpp` dans
Bluefruit. C'est le piège classique de la compilation locale — il ne peut pas
se produire ici.

Si vous compilez aussi sur votre PC et rencontrez cette erreur, déplacez ces
trois dossiers **hors de** `Documents/Arduino/libraries/`.

---

## Version du core

Le workflow fige le core à `Seeeduino:nrf52@1.1.13`, afin que la compilation en
ligne produise le même binaire que la compilation locale. Pour changer de
version, modifiez la variable `CORE` en tête de `.github/workflows/build.yml`
— et pensez à incrémenter le suffixe de la clé de cache pour forcer une
réinstallation propre.
