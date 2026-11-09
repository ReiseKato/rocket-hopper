# ADR 0008: Lageregler außerhalb der physischen Systemschicht

## Status

Akzeptiert

## Kontext

Mit `HopperSystem2D` besitzt der Fast Simulator eine explizite Plant-Grenze.

Diese Systemschicht enthält die physikalischen Aktuatormodelle und die Starrkörpersimulation.

Für die nächste Ausbaustufe wird eine aktive Lageregelung benötigt.

Dabei muss festgelegt werden:

- ob der Controller Bestandteil von `HopperSystem2D` ist,
- welche Größe der Attitude Controller ausgibt,
- wo Aktuatorbegrenzungen angewendet werden,
- wie der kontinuierliche Starrkörperwinkel für die Regelung behandelt wird.

Eine Integration des Controllers in `HopperSystem2D` würde Regler und Plant koppeln und die physische Systemgrenze aufweichen.

Eine direkte Erzeugung eines vollständigen `HopperActuationCommand2D` durch den Attitude Controller würde außerdem die Lageregelung unnötig mit der Schuberzeugung koppeln.

## Entscheidung

Der Attitude Controller wird außerhalb von `HopperSystem2D` implementiert.

Die Regelungsschicht erhält den aktuellen Starrkörperzustand und eine Lagereferenz.

Der Attitude Controller erzeugt ausschließlich den angeforderten Gimbalwinkel:

\[
\delta_{\mathrm{cmd}}.
\]

Ein `CommandProvider` oder eine spätere übergeordnete Regelungsstruktur kombiniert diesen Wert mit einem Schub-Command zu `HopperActuationCommand2D`.

Der Datenfluss lautet:

```text
Attitude Reference
        +
Rigid-Body State
        │
        ▼
AttitudeController2D
        │
        ▼
delta_cmd
        │
        ├── thrust_cmd
        ▼
HopperActuationCommand2D
        │
        ▼
HopperSystem2D
```

Aktuatorgrenzen und Aktuatordynamik bleiben ausschließlich Bestandteil von `HopperSystem2D`.

Der Controller begrenzt seinen Ausgang nicht anhand konkreter Gimbalwinkel- oder Gimbalratenlimits.

`HopperState2D.theta` bleibt als Dynamikzustand ungewrappt.

Nur der für die Regelung verwendete Lagefehler wird auf

\[
[-\pi,\pi)
\]

gewrappt.

Die erste konkrete Implementierung ist ein zustandsloser PD-Regler.

## Konsequenzen

### Positive Konsequenzen

- Regler und Plant bleiben klar getrennt.
- `HopperSystem2D` kann unabhängig vom Controller getestet werden.
- Controller können unabhängig von konkreten Aktuatorimplementierungen entwickelt werden.
- Aktuatorsättigung bleibt im physischen System sichtbar.
- Spätere Höhen- und Positionsregler können unabhängig ergänzt werden.
- Der kontinuierliche Starrkörperwinkel bleibt für Dynamik und Integration unverändert.
- Die Reglerlogik verwendet dennoch den physikalisch kürzesten Winkelfehler.

### Negative Konsequenzen

- Ein `CommandProvider` oder eine spätere übergeordnete Regelungsschicht muss Teil-Commands zu einem vollständigen `HopperActuationCommand2D` zusammensetzen.
- Der einfache PD-Regler kennt die mit dem Schub veränderliche Regelautorität nicht automatisch.
- Bei stark variierendem Schub kann eine spätere Erweiterung wie Gain Scheduling oder Control Allocation notwendig werden.
