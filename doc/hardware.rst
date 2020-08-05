Calculations
============


Saw-Tooth generator
-------------------

.. math::

   V_low = Vcc \cdot \frac{R2 + R3}{R1 + R2 + R3}
   V_high  = (Vcc - U_{CE}) \cdot \frac{R3}{R3 + R4} + U_{CE} - U_{BE}


Known Issues
============

Hardware revision 1
-------------------

- The Saw-Tooth generator has a temperature drift which shifts
  the upper borader of the output signal higher about 5mv/K and
  the lower boarder higher about 2.5mV/K. This results in
  a lowering of the max duty. With 20K Temp shift, the max duty
  reduces about 15% such that typicall panel will enter 
  idle voltage.

