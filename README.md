オムニホイル駆動式のロボットの3Dシミュレータです。[ODE](https://www.ode.org/)を使っています。

# インストール(Liunx)
自分用にメモ

```
$ wget http://prdownloads.sourceforge.net/opende/ode-0.5.tgz
$ tar -xzvf [filename]

$ $cd ode-0.9
＄./configure –enable-double-precision
$   make

$ su
$ make install
```

この後、
```
cd ode/demo（絶対パスでは　cd  /home/ユーザ名/src/ode-0.9/ode/demo)
$./demo_buggy
```
がうまく動けばODEのインストールは成功


で、Makefileで頑張ればmake通った。
