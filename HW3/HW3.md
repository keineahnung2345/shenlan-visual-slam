# HW3

## 群的性质

因为证明$\{\Z, +\}$ 为群时会用到$\{\N, +\}$的部分性质，所以交换了$\{\Z, +\}$，$\{\N, +\}$两题的顺序。

1. $\{\N, +\}$是否为群？若是，验证其满⾜群定义；若不是，说明理由。N 为⾃然数集。 

   1. 封闭性：

      根据[How can one prove that the integers are closed under addition and multiplication? It seems so obvious, but is there a clean proof?](https://www.quora.com/How-can-one-prove-that-the-integers-are-closed-under-addition-and-multiplication-It-seems-so-obvious-but-is-there-a-clean-proof)及[Peano axioms](https://en.wikipedia.org/wiki/Peano_axioms)，有以下的公理：

      ```
      S: single-valued "successor" function S
      
      For every natural number n, S(n) is a natural number. That is, the natural numbers are closed under S.
      ```

      可以把$S$想成$S(x) = x+1$，这个公理是说，$\forall x \in \N, x+1 \in \N$。

      对于任何自然数$a$，可以分以下两种情况证明它与任何自然数$b$的和皆为自然数：

      $\forall a \in \N, b = 0, a+b = a+0 = a \in \N$

      $\forall a \in \N, b \in \Z^+, a+b = a+(1+...+1) = ((((a+1)+1)+1)...+1) \in \N$

   2. 结合律：

      参考[Proofs involving the addition of natural numbers](https://en.wikipedia.org/wiki/Proofs_involving_the_addition_of_natural_numbers#Proof_of_associativity)：

      有以下两条公理：

      ```
      A1:	a + 0 = a
      A2:	a + S(b) = S(a + b)
      ```

      $a_1, a_2, a_3 \in \N$。使用归纳法证明如下：

      - base case：$(a_1 + a_2) + 0 = a_1 + a_2 = a_1 + (a_2 + 0)$

      - 假设$(a_1 + a_2) + a_3 = a_1 + (a_2 + a_3)$，那么$(a_1 + a_2) + S(a_3) = S((a_1 + a_2) + a_3) = S(a_1 + (a_2 + a_3)) = a_1 + S(a_2 + a_3) = a_1 + (a_2 + S(a_3))$

        这代表在假定$a_3$满足结合律的情况下，$S(a_3)$亦满足结合律。所以结合律对任何$a_3 \in \N$皆成立，至此证毕。

   3. 幺元：$0 \in \N, s.t. \forall a \in \N, 0+a = a+0 = a$

   4. 逆：$\forall a \in \Z^{+} \subset \N, -a \notin \N$。故此项不成立。

2. $\{\Z, +\}$ 是否为群？若是，验证其满⾜群定义；若不是，说明理由。Z 为整数集。

   是。

   1. 封闭性：

      对于整数$a$,$b$，分以下几种情况讨论：

      - 对于$b$是0的情况：$a +b = a+0 = a \in \Z$。$a$是0的情况同理可证。

      - $a \in \Z^+ \sub \N, b \in \Z^+ \sub \N$：直接套用自然数对于加法的封闭性。

      - $a \in \Z^-, b \in \Z^-$：

        $-a \in \Z^+, -b \in \Z^+ \Rightarrow (-a) + (-b) = -(a+b) \in \Z^+ \Rightarrow (a+b) \in \Z^-$

      - $a \in \Z^-, b \in \Z^+$：根据Peano Axiom，$\forall x \in \Z^+ \sub \N, x+1 \in \Z^+ \sub \N$。

        扩展到负数，有$\forall x \in \Z^-, S'(x) = x-1 \in \Z^-$，因此：

        $a + b = (-1-1...-1)+(1+1+...1)$

        分以下三种情况讨论：

        - $|a| < |b|$：$a + b = 1+1...+1 = ((((0+1)+1)+1)...+1) \in \Z$
        - $|a| = |b|：a+b = 0 \in \Z$
        - $|a| > |b|：a+b = -1-1...-1 = ((((0-1)-1)-1)...-1) \in \Z$

      - $a \in \Z^+, b \in \Z^-$：同理可证。

   2. 结合律：

      参考[Integer Addition is Associative](https://proofwiki.org/wiki/Integer_Addition_is_Associative)，套用[整数的formal definition](https://en.wikipedia.org/wiki/Integer#Construction)，将三个整数$a_1, a_2, a_3 \in \Z$写成：$a_1 = \left[\!\left[{a, b}\right]\!\right], a_2 = \left[\!\left[{c, d}\right]\!\right], a_3 = \left[\!\left[{e, f}\right]\!\right] $。

      证明如下：

      $\begin{align}a_1 + (a_2 + a_3) &= \left[\!\left[{a, b}\right]\!\right]+( \left[\!\left[{c, d}\right]\!\right]+ \left[\!\left[{e, f}\right]\!\right]) \\&= \left[\!\left[{a, b}\right]\!\right] + \left[\!\left[{c+e, d+f}\right]\!\right] \\&= \left[\!\left[{a+(c+e), b+(d+f)}\right]\!\right] \\&=  \left[\!\left[{(a+c)+e, (b+d)+f}\right]\!\right] \\&= \left[\!\left[{a+c, b+d}\right]\!\right] + \left[\!\left[{e, f}\right]\!\right] \\&= (\left[\!\left[{a, b}\right]\!\right]+\left[\!\left[{c, d}\right]\!\right]) + \left[\!\left[{e, f}\right]\!\right] \\&= (a_1 + a_2) + a_3\end{align}$
   
   3. 幺元：$0 \in \Z, s.t. \forall a \in \Z, 0+a = a+0 = a$
   
   4. 逆：$\forall a \in \Z, \exist -a \in \Z, s.t. a+(-a) = 0$

## 验证向量叉乘的李代数性质

验证$g = (\R^3,\R, \times)$构成李代数

1. 封闭性：

   $X = X_ii+X_jj+X_kk \in \R^3$

   $Y = Y_ii+Y_jj+Y_kk \in \R^3$

   $X \times Y = \begin{vmatrix}i & j & k \\ X_i & X_j & X_k \\ Y_i & Y_j & Y_k\end{vmatrix} = (X_jYk-X_kY_j)i + (X_kY_i-X_iY_k)j + (X_iY_j-X_jY_i)k \in \R^3$

2. 双线性

   参考[Vector Cross Product Operator is Bilinear](https://proofwiki.org/wiki/Vector_Cross_Product_Operator_is_Bilinear)。

   $X = X_ii+X_jj+X_kk \in \R^3$

   $Y = Y_ii+Y_jj+Y_kk \in \R^3$

   $Z = Z_ii+Z_jj+Z_kk \in \R^3$

   $\begin{align}(aX+bY) \times Z &= \begin{vmatrix}i & j & k \\ aX_i+bY_i & aX_j+bY_j & aX_k+bY_k \\ Z_i & Z_j & Z_k\end{vmatrix} \\&= \begin{vmatrix}i & j & k \\ aX_i & aX_j & aX_k \\ Z_i & Z_j & Z_k\end{vmatrix}+\begin{vmatrix}i & j & k \\ bY_i & bY_j & bY_k \\ Z_i & Z_j & Z_k\end{vmatrix} \\&= a\begin{vmatrix}i & j & k \\ X_i & X_j & X_k \\ Z_i & Z_j & Z_k\end{vmatrix}+b\begin{vmatrix}i & j & k \\ Y_i & Y_j & Y_k \\ Z_i & Z_j & Z_k\end{vmatrix}\\&= aX \times Z + bY \times Z \end{align}$

   $\begin{align}Z \times (aX+bY) &= \begin{vmatrix}i & j & k \\ Z_i & Z_j & Z_k \\ aX_i+bY_i & aX_j+bY_j & aX_k+bY_k\end{vmatrix} \\&= \begin{vmatrix}i & j & k  \\ Z_i & Z_j & Z_k\\ aX_i & aX_j & aX_k\end{vmatrix}+\begin{vmatrix}i & j & k  \\ Z_i & Z_j & Z_k \\ bY_i & bY_j & bY_k\end{vmatrix} \\&= a\begin{vmatrix}i & j & k \\ Z_i & Z_j & Z_k \\ X_i & X_j & X_k\end{vmatrix}+b\begin{vmatrix}i & j & k \\ Z_i & Z_j & Z_k \\ Y_i & Y_j & Y_k\end{vmatrix}\\&= aZ \times X +bZ  \times Y \end{align}$

3. 自反性：

   $\begin{align}\begin{matrix}\forall X \in \R^3, X \times X &= \begin{vmatrix}i & j & k \\ X_i & X_j & X_k \\ X_i & X_j & X_k\end{vmatrix} \\&= (X_jX_k-X_jX_k)i + (X_iX_k-X_iX_k)j + (X_iX_j-X_iX_j)k\end{matrix} \\&= 0i + 0j + 0k \\&= 0\end{align}$

4. 雅可比等价

   $\forall X, Y, Z \in V, [X, [Y,Z]]+[Y,[Z,X]]+[Z,[X,Y]] = 0$

   根据[Vector Cross Product satisfies Jacobi Identity](https://proofwiki.org/wiki/Vector_Cross_Product_satisfies_Jacobi_Identity)，套用[Lagrange's Formula](https://proofwiki.org/wiki/Lagrange%27s_Formula)：

   $\begin{align}[X, [Y,Z]]+[Y,[Z,X]]+[Z,[X,Y]] &= (X \times (Y \times Z) + Y \times (Z \times X) + Z \times (X \times Y) \\&= (X \cdot Z)Y - (X \cdot Y)Z \\&+ (Y \cdot X)Z -(Y \cdot Z)X \\&+ (Z \cdot Y)X - (Z \cdot X)Y \\&= 0\end{align}$

## 推导 SE(3) 的指数映射

$\begin{align}\sum\limits_{n=0}^\infty \frac{1}{(n+1)!}(\phi^\text{^})^n\rho &= \sum\limits_{n=0}^\infty \frac{1}{(n+1)!}(\theta a^\text{^})^n\rho \\&= (I + \frac{1}{2!}\theta a^\text{^} + \frac{1}{3!}(\theta a^\text{^})^2 + \frac{1}{4!}(\theta a^\text{^})^3+ \frac{1}{5!}(\theta a^\text{^})^4+...)\rho \\&= ((aa^T-a^\text{^}a^\text{^})+\frac{1}{2!}\theta a^\text{^}+\frac{1}{3!}\theta^2(a^\text{^}a^\text{^})+\frac{1}{4!}\theta^3(-a^\text{^})+\frac{1}{5!}\theta^4(-a^\text{^}a^\text{^})+...)\rho \\&= (aa^T+(\frac{1}{2!}\theta -\frac{1}{4!}\theta^3+...)a^\text{^}+(-1+\frac{1}{3!}\theta^2-\frac{1}{5!}\theta^4+...)a^\text{^}a^\text{^})\rho \\&= (aa^T+\frac{1+(-1+\frac{1}{2!}\theta^2 -\frac{1}{4!}\theta^4+...)}{\theta}a^\text{^}+\frac{(-\theta+\frac{1}{3!}\theta^3-\frac{1}{5!}\theta^5+...)}{\theta}a^\text{^}a^\text{^})\rho \\&= (aa^T+\frac{1-\cos\theta}{\theta}a^\text{^}+\frac{-\sin\theta}{\theta}(aa^T-I))\rho \\&= ((1-\frac{\sin\theta}{\theta})aa^T+\frac{1-\cos\theta}{\theta}a^\text{^}+\frac{\sin\theta}{\theta}I)\rho \\&= J\rho\end{align}$

## 伴随

首先证明$Ra \times Rb = R(a \times b)$，参考[Rotational invariance of cross product](https://math.stackexchange.com/a/1879986/621758)：

$\begin{align}(R(a \times b))_i &= e_i \cdot R(a \times b) \\&= e_i^TR(a \times b) \\&= (R^Te_i)^T \cdot (a \times b) \\&= \det(R^Te_i, a, b) \\&= \det(R)\det(R^Te_i, a, b) \\&= \det(RR^Te_i, Ra, Rb)\\&= \det(e_i, Ra, Rb)\\&= e_i \cdot  (Ra \times Rb)  \\&= (Ra \times Rb)_i\end{align}$

接着利用以上等式来证明$\forall a \in \R^3, Ra^\text{^}R^T = (Ra)^\text{^}$，参考[Derivation of Adjoint for SO(3)](https://math.stackexchange.com/a/2470049/621758)：

令$v \in \R^3$，那么：

$\begin{align}(Ra)^\text{^}v &= (Ra) \times v \\&= (Ra) \times (RR^Tv) \\&= R(a \times R^Tv) \\&= Ra^\text{^}R^Tv\end{align}$

故$(Ra)^\text{^} = Ra^\text{^}R^T$。

接着利用以上等式以及罗德里格公式：$\exp(p^\text{^}) = \exp(\theta a^\text{^}) = \cos\theta I + (1-\cos\theta)aa^T + \sin\theta a^\text{^} = I + (1 - \cos\theta){a^\text{^}}^2 + \sin\theta a^\text{^}$

来证明$SO(3)$的伴随性质：$R\exp(p^\text{^})R^T = \exp((Rp)^\text{^})$。

$\begin{align}R\exp(p^\text{^})R^T &= RIR^T + (1 - \cos\theta)R{a^\text{^}}^2R^T + \sin\theta R a^\text{^}R^T \\&= I + (1 - \cos\theta)R{a^\text{^}}R^TR{a^\text{^}}R^T +\sin\theta (Ra)^\text{^} \\&= I + (1 - \cos\theta)(Ra)^\text{^} +\sin\theta (Ra)^\text{^} \\&= \exp(\theta(Ra^\text{^})) \\&= \exp((Rp)^\text{^})\end{align}$

$SE(3)$的伴随性质：

$T \exp(\xi) T^{-1} = \exp((Ad(T)\xi)^T)$

$Ad(T) = \begin{bmatrix}R & t^\text{^}R \\ 0 & R\end{bmatrix}$

## 轨迹的描绘

1. 事实上， $T_{WC}$的平移部分即构成了机器⼈的轨迹。它的物理意义是什么？为何画出$T_{WC}$的平移
   部分就得到了机器⼈的轨迹？

   $T_{WC}$代表由机器人坐标系到世界坐标系的转换矩阵；$T_{WC}$的平移部分表示由机器人坐标系到世界坐标系的平移向量。$T_{WC}I_4$表示实际对机器人所在的位置做转换，其中$I_4$是机器人的位置在机器人坐标系下的表达，因为机器人相对于自身一定是不动的。$T_{WC}I_4 = T_{WC}$，它的平移部分即机器人在世界坐标系下的位置，画出它也就得到了机器人的轨迹。

2. 完成数据读取部分的代码，然后书写 CMakeLists.txt 以让此程序运⾏起来

   ```cmake
   
   ```

   

## 轨迹的误差

