# Höhenregelung

## Zweck

Die Höhenregelung ergänzt die bestehende Lageregelung um einen zweiten geschlossenen Regelkreis.

Während der Attitude Controller den Gimbal-Command erzeugt, erzeugt der Altitude Controller den Schub-Command.

Die beiden Regler bleiben getrennte Komponenten:

```text
Attitude Reference
        +
HopperState2D
        │
        ▼
AttitudeController2D
        │
        ▼
delta_cmd
        │
        │
        ├───────────────┐
                        │
Altitude Reference      │
        +               │
HopperState2D           │
        │               │
        ▼               │
AltitudeController2D    │
        │               │
        ▼               │
thrust_cmd              │
        │               │
        └───────┬───────┘
                ▼
HopperActuationCommand2D
                │
                ▼
HopperSystem2D
```

Damit bleibt die Regelung modular.

Der Altitude Controller kennt weder die Gimbalregelung noch die konkrete Aktuatorimplementation.

## Regelgrößen

Die Höhenregelung verwendet:

\[
p_z
\]

und:

\[
v_z.
\]

Die Sollwerte werden durch `AltitudeReference2D` beschrieben:

\[
p_{z,\mathrm{ref}}
\]

und:

\[
v_{z,\mathrm{ref}}.
\]

Für stationäres Halten einer Höhe gilt typischerweise:

\[
v_{z,\mathrm{ref}}=0.
\]

## Controller-Schnittstelle

`AltitudeController2D` definiert die gemeinsame Schnittstelle für 2D-Höhenregler.

Ein Controller erhält:

- den aktuellen `HopperState2D`,
- eine `AltitudeReference2D`.

Er gibt ausschließlich den angeforderten Schub zurück:

\[
T_{\mathrm{cmd}}.
\]

Konzeptionell:

\[
(x_{\mathrm{RB}},r_{\mathrm{altitude}})
\rightarrow
T_{\mathrm{cmd}}.
\]

Der Altitude Controller erzeugt keinen Gimbal-Command.

## PD-Regler

Die erste konkrete Implementierung ist `PDAltitudeController2D`.

Der Höhenfehler wird definiert als:

\[
e_z
=
p_{z,\mathrm{ref}}-p_z.
\]

Der Vertikalgeschwindigkeitsfehler lautet:

\[
e_{v_z}
=
v_{z,\mathrm{ref}}-v_z.
\]

Der rohe Schub-Command ergibt sich aus:

\[
T_{\mathrm{cmd,raw}}
=
T_{\mathrm{hover}}
+
K_P e_z
+
K_D e_{v_z}.
\]

Dabei ist:

\[
T_{\mathrm{hover}}=mg
\]

der Feedforward-Anteil für stationären Hover im aufrechten Zustand.

## Bedeutung des Feedforward-Anteils

Ohne Feedforward müsste ein reiner PD-Regler bei:

\[
e_z=0
\]

und:

\[
e_{v_z}=0
\]

einen Schub von:

\[
0
\]

ausgeben.

Das wäre für Hover nicht sinnvoll.

Durch den Feedforward-Anteil gilt im Sollzustand:

\[
T_{\mathrm{cmd}}
=
T_{\mathrm{hover}}.
\]

Im idealen aktuellen Modell wird dafür verwendet:

\[
T_{\mathrm{hover}}=mg.
\]

Der PD-Anteil muss daher nur Abweichungen vom Hoverzustand korrigieren.

## Vorzeichenkonvention

Liegt der Hopper unterhalb seiner Sollhöhe, gilt:

\[
p_z<p_{z,\mathrm{ref}}
\]

und damit:

\[
e_z>0.
\]

Daraus folgt:

\[
T_{\mathrm{cmd}}>T_{\mathrm{hover}}.
\]

Der Hopper erhält zusätzliche Aufwärtsbeschleunigung.

Sinkt der Hopper, gilt bei:

\[
v_{z,\mathrm{ref}}=0
\]

und:

\[
v_z<0
\]

ebenfalls:

\[
e_{v_z}>0.
\]

Auch der D-Anteil erhöht dann den Schub und wirkt der Sinkgeschwindigkeit entgegen.

## Untere Command-Grenze

`HopperActuationCommand2D` erlaubt keinen negativen Schub.

Der PD-Regler kann bei großen negativen Fehlern mathematisch jedoch einen negativen Rohwert erzeugen.

Daher gilt:

\[
T_{\mathrm{cmd}}
=
\max
\left(
0,
T_{\mathrm{cmd,raw}}
\right).
\]

Diese Begrenzung ist keine konkrete physikalische Aktuatorgrenze.

Sie stellt lediglich sicher, dass der Controller einen gültigen `HopperActuationCommand2D` erzeugen kann.

Eine obere Schubgrenze wird bewusst nicht im Controller angewendet.

Der Controller darf daher beispielsweise anfordern:

\[
T_{\mathrm{cmd}}=35\,\mathrm N,
\]

auch wenn das aktuelle Aktuatormodell nur:

\[
T_{\max}=30\,\mathrm N
\]

erzeugen kann.

Die physikalische Begrenzung verbleibt im Schubaktuatormodell.

## Gain-Auslegung

Die Funktion

```python
design_pd_altitude_gains(...)
```

leitet die PD-Gains aus einem gewünschten lokalen System zweiter Ordnung ab.

### Linearisierter Arbeitspunkt

Für einen aufrechten Hopper gilt:

\[
\theta=0
\]

und:

\[
\delta=0.
\]

Die Vertikaldynamik lautet dann:

\[
\ddot p_z
=
\frac{T}{m}
-g.
\]

Mit:

\[
T
=
mg+\Delta T
\]

ergibt sich:

\[
\ddot p_z
=
\frac{\Delta T}{m}.
\]

Für:

\[
\Delta T
=
K_Pe_z
+
K_De_{v_z}
\]

und eine konstante Höhenreferenz ergibt sich lokal die Fehlerdynamik:

\[
\ddot e_z
+
\frac{K_D}{m}\dot e_z
+
\frac{K_P}{m}e_z
=
0.
\]

Verglichen mit:

\[
\ddot e_z
+
2\zeta\omega_n\dot e_z
+
\omega_n^2e_z
=
0
\]

folgt:

\[
K_P
=
m\omega_n^2
\]

und:

\[
K_D
=
2\zeta\omega_n m.
\]

## Beispielauslegung

Für:

\[
m=2\,\mathrm{kg},
\]

\[
\omega_n=2\,\mathrm{rad/s}
\]

und:

\[
\zeta=0.9
\]

ergeben sich:

\[
K_P
=
8\,\mathrm{N/m}
\]

und:

\[
K_D
=
7.2\,\mathrm{N/(m/s)}.
\]

Der Hover-Feedforward beträgt bei:

\[
g=9.81\,\mathrm{m/s^2}
\]

entsprechend:

\[
T_{\mathrm{hover}}
=
19.62\,\mathrm N.
\]

## Kopplung mit der Fahrzeuglage

Die Gain-Auslegung basiert auf einem aufrechten Arbeitspunkt.

Im vollständigen Starrkörpermodell gilt tatsächlich:

\[
\dot v_z
=
\frac{T}{m}
\cos(\theta+\delta)
-g.
\]

Bei geneigtem Schubvektor ist die vertikale Schubkomponente daher kleiner als der Gesamtschub.

Der aktuelle Altitude Controller kompensiert diese geometrische Kopplung nicht explizit.

Er kennt insbesondere nicht den physikalischen Gimbalwinkel.

Stattdessen reagiert er auf die daraus resultierenden Abweichungen in:

\[
p_z
\]

und:

\[
v_z.
\]

Für die erste Ausbaustufe ist dies bewusst ausreichend, weil die Lageregelung den Hopper schnell wieder in die Nähe des aufrechten Arbeitspunkts führt.

Eine spätere Regelungsarchitektur kann bei Bedarf eine explizite Neigungskompensation oder eine gekoppelte Control-Allocation-Struktur ergänzen.

## Zusammenspiel mit der Lageregelung

Im ersten kombinierten Closed-Loop-Versuch arbeiten beide Regler gleichzeitig.

Die Lageregelung erzeugt:

\[
\delta_{\mathrm{cmd}}.
\]

Die Höhenregelung erzeugt:

\[
T_{\mathrm{cmd}}.
\]

Ein `CommandProvider` kombiniert beide Größen:

```python
return HopperActuationCommand2D(
    thrust=thrust_command,
    gimbal_angle=gimbal_command,
)
```

Dadurch bleibt die Verantwortung eindeutig verteilt.

## Horizontaler Drift

Der Altitude Controller regelt keine horizontale Bewegung.

Auch wenn:

\[
p_z\rightarrow p_{z,\mathrm{ref}}
\]

und:

\[
v_z\rightarrow0
\]

erreicht werden, kann weiterhin gelten:

\[
v_x\neq0.
\]

Der Hopper driftet dann horizontal weiter.

Dies ist im aktuellen Entwicklungsstand erwartetes Verhalten.

Die horizontale Bewegung wird erst durch einen späteren äußeren Positionsregler geschlossen.

## Erster kombinierter Closed-Loop-Versuch

Das Beispiel:

```text
attitude_altitude_stabilization.py
```

startet mit:

\[
p_{z,0}=2\,\mathrm m,
\]

\[
v_{z,0}=0,
\]

\[
\theta_0=5^\circ,
\]

\[
\omega_0=0.
\]

Die Sollwerte lauten:

\[
p_{z,\mathrm{ref}}=2\,\mathrm m,
\]

\[
v_{z,\mathrm{ref}}=0,
\]

\[
\theta_{\mathrm{ref}}=0,
\]

\[
\omega_{\mathrm{ref}}=0.
\]

Die anfängliche Lagekorrektur kann einen kleinen Höhen- und Geschwindigkeitsfehler erzeugen.

Die Höhenregelung soll diesen anschließend wieder abbauen.

Erwartet wird:

\[
\theta\rightarrow0,
\]

\[
\omega\rightarrow0,
\]

\[
p_z\rightarrow2\,\mathrm m,
\]

\[
v_z\rightarrow0.
\]

Der horizontale Zustand bleibt bewusst ungeregelt.

## Grenzen der ersten Version

Die aktuelle Höhenregelung besitzt noch keine:

- Integralregelung,
- explizite Neigungskompensation,
- Anti-Windup-Logik,
- Schubdynamik,
- Gain-Scheduling,
- Trajektorienregelung,
- Beschleunigungsreferenz,
- Störgrößenbeobachtung,
- Messfehler oder Sensorverzögerung.

Diese Erweiterungen werden erst ergänzt, wenn der einfache kombinierte Closed-Loop-Versuch validiert wurde.
