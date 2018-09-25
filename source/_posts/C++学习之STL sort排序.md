---
title: C++学习之STL sort排序
date: 2018-04-12 11:41:08
tags:
  - STL
categories: 
  - 语言
  - C++
---

-----

这篇文章是有关C++ STL sort的学习内容。

<!--more--->

`std::sort`函数是C++ STL中自带的排序函数，该函数对给定区间所有元素进行排序。函数原型为：

~~~c++
//default (1)	使用了函数模板
template <class RandomAccessIterator>
  void sort (RandomAccessIterator first, RandomAccessIterator last);
//custom (2)	
template <class RandomAccessIterator, class Compare>
  void sort (RandomAccessIterator first, RandomAccessIterator last, Compare comp);
~~~

## 用法

- 需引用`#include <algorithm>`、`using namespace std`；
- 使用类似快速排序的方法，时间复杂度为`n*logn`；
- 默认的`sort`函数有两个参数，也可以使用三个参数：
  - 参数`first`是要排序的数组的起始地址
  - 参数`last`是数组结束的地址（最后一位要排序的地址），排序范围：[first, last)
  - 参数`comp`是排序的方法，返回值为`bool`类型，可以升序或降序。可省略，此时默认的排序方法是升序。

## 举例

~~~c++
/ sort algorithm example
#include <iostream>     // std::cout
#include <algorithm>    // std::sort
#include <vector>       // std::vector

bool myfunction (int i,int j) { return (i<j); }　//升序
bool myfunction１ (int i,int j) { return (i>j); }//降序

struct myclass {
  bool operator() (int i,int j) { return (i<j);}//运算符重载????
} myobject;

int main () {
  int myints[] = {32,71,12,45,26,80,53,33};
  std::vector<int> myvector (myints, myints+8);               // 32 71 12 45 26 80 53 33

  // using default comparison (operator <):
  std::sort (myvector.begin(), myvector.begin()+4);           //(12 32 45 71)26 80 53 33

  // using function as comp
  std::sort (myvector.begin()+4, myvector.end(), myfunction); // 12 32 45 71(26 33 53 80)

  // using function as comp
  std::sort (myvector.begin()+4, myvector.end(), myfunction1); // 12 32 45 71(80 53 33 26)

  // using object as comp
  std::sort (myvector.begin(), myvector.end(), myobject);     //(12 26 32 33 45 53 71 80)

  // print out content:
  std::cout << "myvector contains:";
  for (std::vector<int>::iterator it=myvector.begin(); it!=myvector.end(); ++it)
    std::cout << ' ' << *it;
  std::cout << '\n';

  return 0;
} 
~~~

## 另一个例子

~~~c++
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

struct Edge
{
  int vertexStart; //边连接的一个顶点编号
  int vertexEnd; //边连接另一个顶点编号
  int vertexWeight; //边的权值
  friend bool operator<(const Edge& E1, const Edge& E2)//友元函数
  {
      return E1.vertexWeight < E2.vertexWeight;
  }
};

// bool operator<(const Algorithm::Edge& E1, const Algorithm::Edge& E2)
// {
//     return E1.vertexWeight < E2.vertexWeight;
// }
// 
// bool compare_edge(const Algorithm::Edge& E1, const Algorithm::Edge& E2)
// {
//   return E1<E2;
// }

int main(int argc ,char **argv)
{
  vector<Edge> edge;
  edge.assign(10, Edge());
  for (int i = 0; i < 10; i++)
      edge[i].vertexStart = edge[i].vertexEnd = edge[i].vertexWeight = i;
  sort(edge.begin(), edge.end());
  //sort(edge.begin(), edge.end(), compare_edge);
  return 1;
 }
~~~

可以使用在结构体（或者是类）声明的友元函数运算符重载函数，这时调用两个参数的sort函数即可；

也可以在结构体定义外面单独声明运算符重载函数、比较函数，要使用三个参数的sort函数，第三个参数传入比较函数。

