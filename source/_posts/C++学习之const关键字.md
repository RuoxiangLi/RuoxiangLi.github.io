---
title: C++学习之const关键字
date: 2018-03-23 21:39:43
tags:
  - C++
  - 面试
categories: 
  - 语言
  - C++
copyright: true
---

------

这篇文章是有关C++ const关键字的学习内容。

<!--more--->

# const介绍与分类

`const`是constant的简写，只要一个变量前面用`const`来修饰，就意味着该变量里的数据可以被访问，不能被修改。也就是说const意味着“只读”readonly。

采用符号常量写出的代码更容易维护；指针常常是边读边移动，而不是边写边移动；许多函数参数是只读不写的。const最常见用途是作为数组的界和switch分情况标号(也可以用枚举符代替)，const的使用分类如下：

- 常变量：  **const 类型说明符 变量名**
- 常引用：  **const 类型说明符 &引用名**
- 常对象：  **类名 const 对象名**
- 常成员函数：  **类名::fun(形参) const**
- 常数组：  **类型说明符 const 数组名[大小]**    
- 常指针：
  - **const 类型说明符\* 指针名 ** (指针称为指针常量，指针指向是一个常量)
  - **类型说明符* const 指针名**  (指针称为常量指针，指针本身是一个常量)

注意：在常变量、常引用、常对象、常数组，const 与 “类型说明符”或“类名”（其实类名是一种自定义的类型说明符）的位置可以互换，但在常指针中不能互换，这一点在下面的内容也会提到。

# const的作用

1. 如果想阻止一个变量被改变，可以使用`const`关键字。在定义该const变量时，通常需要对它进行初始化，因为以后就没有机会再去改变它了；例如`const int a = 3; const int & b = a;`必须向上面的方式写进行初始化，但`const int * p; int a = 3; p = &a; `，这样也是可以的，当然最好将指针常量初始化为`Null`。

2. 对指针来说，可以指定指针本身为`const`，也可以指定指针所指的数据为`const`，或二者同时指定为`const`；

   例如：

   ~~~c++
   int b = 500;
   const int * a = &b;       //1
   int const * a = &b;       //2
   int * const a = &b;       //3
   const int * const a = &b; //4
   ~~~

   分析：

   - `const`位于`*`左侧，`const`修饰指针所指向的变量，即指针指向常量，不能修改指针指向的内容，p称为指针常量；
   - `const`位于`*`右侧，`const`修饰指针本身，即指针本身是常量，不能对指针本身进行更改操作，p是常量指针。

   因此，1和2的情况相同，都是指针所指向的内容为常量（`const`放在变量声明符的位置无关），这种情况下不允许对内容进行更改操作，如不能`*a = 3` ；

   3表示指针本身是常量，而指针所指向的内容不是常量，这种情况下不能对指针本身进行更改操作，如`a++`是错误的；

   4为指针本身和指向的内容均为常量。

3. 在一个函数声明中，`const`可以修饰形参，表明它是一个输入参数，在函数内部不能改变其值；

   例如：

   ~~~c++
   void fun0(const A * a );
   void fun1(const A & a); 
   ~~~

   调用函数的时候，用相应的变量初始化const常量，则在函数体中，按照const所修饰的部分进行常量化，如形参为｀`const A* a`，则不能对传递进来的指针的内容进行改变，保护了原指针所指向的内容；如形参为`const A& a` ，则不能对传递进来的引用对象进行改变，保护了原对象的属性。

   注意：参数`const` 通常用于参数为指针或引用的情况。

   ​

4. 对于类的成员函数，若指定其为`const`类型，则表明其是一个常函数，不能修改类的成员变量；`const` 一般放在函数体后，形如：`void fun() const;`。 如果一个成员函数的不会修改数据成员，最好将其声明为`const`，因为`const`成员函数中不允许对数据成员进行修改，如果修改，编译器将报错，这大大提高了程序的健壮性。

   **知识扩展：**如果有在`const`函数中修改成员变量值的需求，可以把成员变量声明为`mutable`类型，声明为`mutable`类型的成员变量可以在常函数中被修改。`mutable`是为了突破const的限制而设置的，被`mutable`修饰的变量，将永远处于可变的状态，即使在一个const函数中。可以[参考博客](https://blog.csdn.net/starlee/article/details/1430387)　理解。

5. 对于类的成员函数，有时候必须指定其返回值为`const`类型，以使得其返回值不为"左值”。

   例如：

   `const classA operator*(const classA & a1, const classA & a2);`

   `operator*`的返回结果必须是一个`const`对象。需要注意，如果返回值不是`const`对象也不会编译出错。

   ```c++
   classA a, b, c;
   (a * b) = c; // 对a*b的结果赋值
   ```

   操作`(a * b) = c`显然不符合编程者的初衷，也没有任何意义。

   ​

   **注意：**

   - `const` 返回类型只有在修饰指针或引用是才有用。

   - `const`对象只能调用类的`const`函数，因为常量对象的状态不允许被修改，调用常量对象的非常量函数会出错

   - 在类中只有`const`函数时，非`const`对象可以调用`cosnt`函数，但有非`const`函数时，非`const`对象不能调用`const`函数。

   - 非`const`函数、`const`函数同时存在时，非常对象将调用非常函数，常对象调用常函数。

     例如：

     ~~~c++
     #include <iostream>
     using namespace std;

     class A
     {
         public:
             A(int v): val(v) {}
             
             void print_val() { cout << "not const:" << val << endl;}
             void print_val() const { val++; cout << "const print_val:" << val << endl;}
             
         private:
             mutable int val;//注意体会mutable的作用
     };

     int main(int argc ,char **argv)
     {
         A b(45);
         b.print_val();

         const A a(12);
         a.print_val();
     }
     ~~~

     输出结果为：

     ~~~
     not const:45
     const print_val:13
     ~~~

6. 类的成员变量为`const`类型时，类的const成员变量的初始化，必须在构造函数的初始化列表中进行。初始化列表是先于构造函数的函数体执行，并且成员初始化列表中的变量初始化顺序与成员在类中的声明顺序相同。

   **注意：**

   - `const`修饰的局部变量在栈上分配内存空间，`const`修饰的全局变量在只读存储区分配内储存空间。
   - **类的static　const成员变量可以在类的内部声明时初始化**

# const的初始化

1. 非指针`const`常量初始化

   ~~~c++
   classA b;
   const classA a = b;
   ~~~

2. 指针(引用)const常量初始化

   - 指针

   ```c++
   classA * d = new classA();
   const classA * c = d;
   ```

   或

   ```c++
   const classA * c = new classA();
   ```

   - 引用

   ```c++
   classA f;
   const classA　& e = f; //这种方式，e只能访问声明为const的成员函数，不能访问一般的成员函数

   ```

3. 类的const成员变量初始化

   ~~~c++
   class A  
   {  
   public:   
          A():Size(0){}//必须在构造函数的初始化列表初始化
          
   private:
         const int Size;
   }  
   ~~~

# 思考题

1. 以下的这种赋值方法正确吗？

    ```c++
       const classA * c = new classA();
       classA * e = c; 
    ```


2. 以下的这种赋值方法正确吗？

   ```c++
   classA * const c = new classA();
   classA * b = c;
   ```

3. 这样定义赋值操作符重载函数可以吗？

    ~~~c++
      const classA & operator = (const classA & a);
    ~~~

[思考题答案]

1. 不正确。因为声明指针的目的是为了对其指向的内容进行改变，而声明的指针`e`指向的是一个常量，所以不正确。

2. 正确。因为声明指针所指向的内容可变。

3. 不正确。在`const A::operator=(const A& a)`中，参数列表中的`const`的用法正确，而当这样连续赋值的时侯，问题就出现了：

   ~~~c++
   A a,b,c:
   (a = b) = c;
   ~~~

   因为`a.operator = (b)`的返回值是对`a`的`const`引用，不能再将`c`赋值给`const`常量。