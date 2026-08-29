# Defectos Conocidos y Misterios (GPS Tracker Logan)

*Registro de contexto y deuda técnica para futuros desarrollos y agentes de IA.*

## Defectos conocidos y NO arreglados

### Los que molestan de verdad:
* **Parqueado, el tracker es invisible:** El repaso de 2 minutos solo publica en la rama del pulso de 24 h. La noche del 22 hubo ~275 despertares mudos en 9 horas. Funciona, pero no lo puedes ver funcionar.
* **La ventana de 60 s no alcanza:** Cada despertar da 60 segundos de gracia y el primer fix en la calle tarda 3–4 minutos. El repaso, por diseño, nunca va a conseguir posición.
* **F20, la velocidad en nudos:** La aritmética ya lo dio por cerrado (razón 1,861 ≈ 1,852), pero los umbrales del firmware siguen mintiendo: `MOVING_SPEED_KMH` 3.0 son 5,6 km/h reales y el techo de 180 son 333. Falta confirmar el campo en el manual §21.2.
* **El hardware del sense está mal dimensionado:** Divisor 47k/68k con el capacitor del lado equivocado, buck a 5,25 V, umbral a 2,5 V y el Zener fugando 13 µA sobre un divisor de 40. *Pendiente:* el capacitor de 10–100 nF directo del pin a tierra, y el rediseño a 10k/12k con umbral a 2,2 V, que exige recalibrar `DIVIDER_FACTOR`.

### Los que están agachados esperando su turno:
* `pmModemWake()` sin plan B: 15 intentos de AT y si no contesta, sigue como si nada hubiera pasado.
* Después de `restartModem()` nadie vuelve a encender el receptor de satélites. Queda apagado hasta el próximo reset.
* El hold del DTR solo se re-arma dentro de `pmModemSleep()`.
* `CGNSSMODE` se reescribe en cada arranque siendo un ajuste que el módem ya guarda solo.
* La zona media del pin (`ign_sense_midzone`) solo se detecta despierto.
* Socket zombi `state=-4`.
* Polaridad de `PWRKEY` invertida en `at_passthrough.cpp` — solo estorba en laboratorio.
* El tag `v2.0-lowpower-validado` apunta a `704c0ad` y no al commit que se validó. Se dejó así a propósito.

---

## Comportamientos que NO hemos logrado explicar

| Misterio | Por qué sigue abierto |
| :--- | :--- |
| **Los diez minutos** | Si el bucle de `enableGPS()` corría sus 610 s completos, deberíamos haber visto diez minutos de silencio. Vimos 3 min 29 s. Nunca cerró. |
| **Quién pellizca el pin** | Módem o carro. La única pista es que los intervalos fueron 1 h 34 min y 3 h 08 min — exactamente el doble. El `spur=` de mañana es lo que va a decidir. |
| **El silencio de 61 s y otro de 119 s antes de un apagado** | Sospecho `IDLE_PERIOD_MS` (30 s) peleando con `MOVING_HOLD_MS` (60 s), pero no lo he leído. |
| **El `engine_off` de 21:39:20Z** | Con ese log no se puede distinguir de un apagado normal. |
| **A-GNSS** | `AT+CAGPS` contestó OK y empeoró el arranque a 192 s con 9 satélites. El comando documentado es `CGNSSAGPS` y nunca se ha probado. Tu duda sigue viva y con razón. |
| **Dormir vs apagar** | No hay un solo dato que cierre esto. Tu escepticismo sigue en pie. |
| **CGNSSSLEEP** | n=1 y salió peor (45 s contra 22 s). No se puede concluir nada. |
| **"Terror Gris"** | Sigo sin saber por qué Traccar llama así al dispositivo si el broker publica en `tracker/Lilygo/`. Tercera vez que pregunto. |

