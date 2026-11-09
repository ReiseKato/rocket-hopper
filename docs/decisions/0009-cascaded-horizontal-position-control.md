# ADR 0009: Horizontale Positionsregelung als äußerer Attitude-Regelkreis

## Status

Akzeptiert

## Kontext

Der 2D-Hopper besitzt keinen unabhängigen horizontalen Aktuator.

Die horizontale Beschleunigung entsteht durch die horizontale Komponente des Schubvektors:

\[
\dot v_x
=
\frac{T}{m}
\sin(\theta+\delta).
\]

Ein Horizontal Position Controller könnte grundsätzlich direkt einen Gimbal-Command erzeugen.

Dadurch würden jedoch zwei unterschiedliche Aufgaben vermischt:

1. Bestimmung der für die horizontale Bewegung benötigten Fahrzeuglage,
2. Regelung der Fahrzeuglage über den Gimbal.

Die zweite Aufgabe wird bereits vom bestehenden `AttitudeController2D` übernommen.

Zusätzlich muss festgelegt werden, wie aggressive horizontale Commands begrenzt werden.

## Entscheidung

`HorizontalPositionController2D` wird als äußerer Regelkreis implementiert.

Er erhält:

- `HopperState2D`,
- `HorizontalPositionReference2D`.

Er erzeugt:

- `AttitudeReference2D`.

Der Datenfluss lautet:

```text
HorizontalPositionReference2D
        +
HopperState2D
        │
        ▼
HorizontalPositionController2D
        │
        ▼
AttitudeReference2D
        │
        ▼
AttitudeController2D
        │
        ▼
delta_cmd
```

Der Horizontal Position Controller erzeugt keinen Gimbal-Command und keinen Schub-Command.

Die erste Implementierung erzeugt zunächst eine horizontale Beschleunigungsanforderung:

\[
a_{x,\mathrm{cmd}}
=
K_Pe_x
+
K_De_{v_x}.
\]

Eine maximale zulässige Lagereferenz \(\theta_{\max}\) definiert die maximale horizontale Beschleunigungsanforderung:

\[
a_{x,\max}
=
g\sin(\theta_{\max}).
\]

Der Beschleunigungs-Command wird auf diesen Bereich begrenzt.

Anschließend wird die Lagereferenz über:

\[
\theta_{\mathrm{ref}}
=
\arcsin
\left(
\frac{a_{x,\mathrm{limited}}}{g}
\right)
\]

bestimmt.

Damit erfolgt die Begrenzung konsistent mit derselben physikalischen Abbildung, die für die Erzeugung der Lagereferenz verwendet wird.

## Konsequenzen

### Positive Konsequenzen

- Die bestehende Lageregelung wird wiederverwendet.
- Horizontal- und Attitude-Regelung bleiben getrennt testbar.
- Die Regelungsstruktur entspricht einer klaren Kaskade aus äußerem Positions- und innerem Lagekreis.
- Der äußere Regelkreis erzeugt keine aktuatorkonkreten Größen.
- Die maximale Lagereferenz ist explizit und physikalisch interpretierbar.
- Die horizontale Reglerbandbreite kann unabhängig vom inneren Attitude Loop gewählt werden.

### Negative Konsequenzen

- Die äußere Regelung hängt davon ab, dass der innere Attitude Loop ausreichend schnell ist.
- Die aktuelle Abbildung \(a_x\approx g\sin\theta\) gilt nur näherungsweise um einen Hover-Arbeitspunkt mit \(T\approx mg\).
- Änderungen des tatsächlichen Schubs beeinflussen die reale horizontale Beschleunigung.
- Stark gekoppelte Manöver können später eine explizitere Control-Allocation- oder Beschleunigungsregelung erforderlich machen.
