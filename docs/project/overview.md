# Projektüberblick

Der Rocket Hopper soll langfristig eine persönliche Forschungs- und Entwicklungsplattform für wiederverwendbare Raketenlander-ähnliche Systeme werden.

Die bevorzugte Grundarchitektur besteht aus einem länglichen Fahrzeugkörper, einem zentralen Hauptantrieb und einem 2-Achs-Gimbal zur Schubvektorsteuerung. Konkrete Hardware ist derzeit noch nicht festgelegt. Masse, Abmessungen, Trägheitsmomente, Antrieb, Servos und Batterie werden deshalb nicht als bekannte Wahrheiten behandelt.

## Digital-Twin-Gedanke

Der Simulator beginnt mit angenommenen Parametern. Später werden diese Annahmen schrittweise durch Messungen ersetzt:

```text
Simulation
    ↓
Hardwaretest
    ↓
Messdaten
    ↓
Systemidentifikation
    ↓
Modellupdate
    ↓
Validierung
    ↺
```

Der Fast Simulator und ein späteres MuJoCo-Backend sollen unterschiedliche Aufgaben erfüllen. Der Fast Simulator bleibt für transparente Dynamik, schnelle Parameterstudien, Optimierung und Monte-Carlo-Simulation zuständig. MuJoCo wird später insbesondere für Kontakte, Landebeine, Reibung und komplexe mechanische Interaktionen eingesetzt.
