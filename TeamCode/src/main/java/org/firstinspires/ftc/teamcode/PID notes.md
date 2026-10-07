P -> present

I -> past

D -> Future

##### **Error**

names for the Difference between desired and actual variable.

* &#x20;e(t)
* (SV - SP)
* Error
* any combination of the above

* goal is to minize e(t). in other words bring actual to the Setpoint
* control variable u(t) is changed by P, I, and D
* you can choose to not use P, I or, D by setting the number in the equation to 0
* e(t) is accessing the error at the current time, e(t) returns a value but IS a function. 



##### **Derivatives**

* derivative looks at e(t) and calculates the derivative 



de(t)

\----- takes e(t) and spits out a function to calculate the derivative

dt 



##### **Control function explanation**

three terms. Kp Ki Kd. all added together. each of those represents P, I, and D. For example the Ki is paired with an integration function to integrate e(t) from time 0 -> current. 

