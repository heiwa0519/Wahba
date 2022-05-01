<center>
<h4>A Least Squares Estimate of Satellite Attitude</h4>
Grace Wahba</br>
1965</br>
</center>

Given two sets of $n$ points $\{\vec{r}_{1}, \vec{r}_{2}, ..., \vec{r}_{n}\}$, and $\{\vec{y}_{l}, \vec{y}_{2}, ... , \vec{y}_{n}\}$, where $n \geq 2$, find the rotation matrix $\bm{Q}$ (i.e., the orthogonal matrix with determinant +1) which brings the first set into the best least squares coincidence with the second. That is, find $\bm{Q}$ which minimizes

$$\sum_{j=1}^{n} ||\vec{y}_{j} - \bm{Q} \vec{r}_{j}||^{2}.$$
