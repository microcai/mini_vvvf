# mini vvvf

市面上能买到的异步电机变频控制器都是380v电压的. 不太适合搞实验.
于是我改造了一下 simplefoc 项目, 让他可以用来当变频器.

## 原理

其实就是 SVPWM . 市面上的无刷电机控制器, 只要是 foc 的, 其实理论上都支持交流异步电机.
但是, 他们的软件都多做了异步, 试图与转子同步. 还好 simplefoc 项目开源, 于是硬件可以使
用 现有的, 软件上则少做点活, 不和转子同步, 不就能控制了.

## 异步电机变频控制原理

通过改变交流电的频率实现调速. 变频的同时需要变压. 使用固定的 电压/频率 比值.
这个原因其实是因为, 降低频率后, 电机转速降低, 的反电动势也会降低, 而且交流电频率下降后,
绕组的阻抗也下降了, 如果不降低电压, 会导致电机的电流过大, 导致磁通饱和, 过大的电流只能
变成热量, 完全无法体现在扭矩的提升上.

降频后降压, 是防止电机磁通饱和, 降低发热, 提高效率.

