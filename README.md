# GSM alarm (managed by SMS)
This is my first project for getting acquainted with and developing skills in low-level microcontroller programming.
Bellow you can find first public version of GSM alarm device<br>
A detailed description of the project, key device features, and control commands can be found at the following link:  https://mysku.club/blog/diy/94054.html (RU version) you can use google translate for another language.

Example of existing commands. You can send multiple commands in one SMS.

<code>Command:Value - Description</code>
<pre>SMS:0 - do not send sms status
    1 - send only to the primary number (by default).
    2 - for both
S0:ON - activate the zero sensor to detect an alarm
   OFF - turn it off
S1:ON - activate the first sensor
   OFF - turn it off
S2:ON - activate the second sensor
   OFF - turn it off
S3:ON - activate the third sensor
   OFF - turn off
all sensors are turned on by default.
S0LV:HI - logic level of the zero sensor when there is no alarm set as high
     LO - how low
S1LV:HI - logical level of the first sensor when there is no alarm set as high
     LO - how low
S2LV:HI - logical level of the second sensor when there is no alarm set as high
     LO - how low
S3LV:HI - logic level of the third sensor when there is no alarm set as high
     LO - as low
by default, all sensors are low.
CALL:0 - do not call when there is an alarm
     1 - call only the first one (by default).
     2 - for both
BELL:ON - activate the pin of the external siren (or other device) [ON].
     OFF - do not activate [OFF]. by default.
BPLV:HI - the logical level of the external siren when there is an alarm is set as high. by default.
     LO - how low
BELLT:10..180 - the time in seconds during which the pre-set logic level will be applied to the pin of the external siren. 30 by default.
BEEP:0..9 - how many conditional rings to call the numbers. the default is 3.
ADMIN:a number without a local area code. can send commands
ADMIN2:a number without a local area code. The length is negative as in the first one.
ALARM:ON - activate the alarm. by default.
      OFF - turn it off
SLEEP:ON - activate sleep mode.
      OFF - turn it off. by default.
POWER:ON - activate the alarm mode when the 220 mains is lost. if battery power is not needed only.
      OFF - turn it off
DELAY:0..250 is the delay time before the alarm is triggered in seconds. after how long will the alarm be triggered when the sensors have already detected the penetration. 0 by default. i.e. without delay immediately.
GETBAL:ON - receive the balance of the SIM card in the report.
       OFF - turn it off. by default.
BALNUM:xxx is the number from which to receive the balance in USSD mode, for example *100#
AWAKET:3..60 - how many minutes to go to guard / sleep and how many minutes to wait for an SMS command from the admin. 3 by default.
DELTEL:1 - delete the admin number and the second number.
       2 - delete only the second number
PREFIX:0 - 0 is added before the number. by default.
       1 - +7
       2 - +373
       3 - +375
       4 - +380
RESETA - arduino reset
RESETC - config reset
BALANCE - send an SMS report to the admin now</pre>
