---
title: C++中使用YAML语言
date: 2018-05-23 09:20:12
tags:
  - C++
  - YAML
  - ROS
  - OpenCV
categories: 
  - 语言
  - YAML
copyright: true
---

-----

这篇文章是有关C++中使用YAML语言的基础，以及在结合ROS和OpenCV使用的学习内容。

<!--more--->

## YAML语言使用基础

YAML语言在C++中，可以用于应用程序相关配置文件的保存，可读性、可修改性比较好。直接贴代码。

测试代码：

~~~c++
#include <fstream>
#include <iostream>
#include <string>
#include <assert.h>
#include "yaml-cpp/yaml.h"

using namespace std;

//=============================================================================
void test1()
{
    YAML::Node config = YAML::LoadFile("site.yaml");
 
    std::cout<<"here test1!!"<<endl;
    YAML::Node config_systemLog = config["systemLog"];
    
    if ( config_systemLog["path"] )
        std::cout << config_systemLog["path"] << std::endl;//输出c:\data\log\mongod.log
 
    std::ofstream fout( "test1.yaml" );
    fout << config;
}
 
//=============================================================================
void test2()
{
    YAML::Node node;
    node["username"] = "glimix";
    node["password"] = "111222";
 
    std::ofstream fout( "test2.yaml" );
    fout << node;
}
 
//=============================================================================
void test3()
{    
    try
    {
        YAML::Node doc = YAML::LoadFile( "Night.jpg.meta" );
        std::cout << doc << "\n";
    }
    catch( const YAML::Exception &e )
    {
        std::cerr << e.what() << "\n";//读取文件异常
    }
}
 
//=============================================================================
void test4()
{
    YAML::Node node;
    node.push_back( "glimix" );
    node.push_back( 123 );
    node.push_back( 3.1415926 );
    node["char"].push_back( 'a' );
    node["bool"].push_back( true );
 
    std::ofstream fout( "test4.yaml" );
    fout << node;
}
 
//=============================================================================
void test5()
{
    class Vector3
    {
    public:
        float x, y, z;
 
        void encode( YAML::Node &node )
        {
            node.push_back( x );
            node.push_back( y );
            node.push_back( z );
        }
 
        bool decode( YAML::Node &node )
        {
            if ( !node.IsSequence() || node.size() != 3 )
                return false;
 
            x = node[0].as<float>();
            y = node[1].as<float>();
            z = node[2].as<float>();
 
            return true;
        }
    };
 
    Vector3 pos;
    pos.x = 100.0f; pos.y = -45.0f; pos.z = 50.0f;
     
    YAML::Node node;
    pos.encode( node );
 
    std::ofstream fout( "test5.yaml" );
    fout << node;
 
    Vector3 pos2;
    pos2.decode( node );
}
 
//=============================================================================
void test6()
{
    YAML::Node node;
    node["name"] = "John Smith";
    node["age"] = 37;
     
    YAML::Node node2;
    node2["name"] = "Jane Smith";
    node2["age"] = 25;
     
    YAML::Node node3;
    node3["name"] = "Jimmy Smith";
    node3["age"] = 15;
    
    YAML::Node node4;
    node4["name"] = "Jenny Smith";
    node4["age"] = 12;
 
    node["spouse"] = node2;
    node["children"].push_back( node3 );
    node["children"].push_back( node4 );
 
    node["children"].push_back( "{name: Alex Smith, age: 14}" );
    node["children"].push_back( "name: Alex Smith, age: 14" );
    node["children"].push_back( YAML::Load( "{name: Alex Smith, age: 14}" ) );
    YAML::Node node5 = YAML::Load( "{name: Alex Smith, age: 14}" );
    node["children"].push_back( node5 );
 
    std::ofstream fout( "test6.yaml" );
    fout << node;
}
 
//=============================================================================
void test7()
{
    YAML::Node node;  // starts out as null
    node["key"] = "value";  // it now is a map node
    node["seq"].push_back("first element");  // node["seq"] automatically becomes a sequence
    node["seq"].push_back("second element");
 
    node["mirror"] = node["seq"][0]; // this creates an alias
    node["seq"][0] = "1st element";  // this also changes node["mirror"]
    node["mirror"] = "element #1";   // and this changes node["seq"][0] - they're really the "same" node
 
    node["self"] = node;  // you can even create self-aliases
    node[node["mirror"]] = node["seq"];  // and strange loops 🙂
 
     
    std::ofstream fout( "test7.yaml" );
    fout << node;
}

//=============================================================================
void test8()
{
  YAML::Node primes = YAML::Load("[2, 3, 5, 7, 11]");
  for (std::size_t i=0;i<primes.size();i++) 
  {
   std::cout << primes[i].as<int>() << "\n";
  }
  // or:
//   for (YAML::const_iterator it=primes.begin();it!=primes.end();++it) 
//   {
//     std::cout << it->as<int>() << "\n";
//   }

  primes.push_back(13);
  assert(primes.size() == 6);
  std::ofstream fout("test8.yaml");
  fout << primes;
}

//=============================================================================
void test9()
{
 YAML::Node lineup = YAML::Load("{1B: Prince Fielder, 2B: Rickie Weeks, LF: Ryan Braun}");
 for(YAML::const_iterator it = lineup.begin(); it != lineup.end(); ++it) 
 {
   std::cout << "Playing at " << it->first.as<std::string>() << " is " << it->second.as<std::string>() << "\n";
 }

 lineup["RF"] = "Corey Hart";
 lineup["C"] = "Jonathan Lucroy";
 assert(lineup.size() == 2);//assert(表达式)保证满足特定条件，即表达式为真，否则整个程序退出并输出一条错误信息。
 
 std::ofstream fout("test9.yaml");
  fout << lineup;
}

//=============================================================================
void test10()
{
 YAML::Node node = YAML::Load("{name: Brewers, city: Milwaukee}");

 if (node["name"]) {
   std::cout << node["name"].as<std::string>() << "\n";
 }
 if (node["mascot"]) {//无输出
   std::cout << node["mascot"].as<std::string>() << "\n";
 }
 assert(node.size() == 2); // the previous call didn't create a node
}

//=============================================================================
void test11()
{
  YAML::Node node1 = YAML::Load("[1, 2, 3]");//序列结构
  assert(node1.Type() == YAML::NodeType::Sequence);
  assert(node1.IsSequence());  // a shortcut!
  
  YAML::Node node2 = YAML::Load("{name: Brewers, city: Milwaukee}");//散列/map结构
  switch (node2.Type()) 
  {
   case YAML::NodeType::Null: cout<<"NodeType:NULL"<<endl; break;
   case YAML::NodeType::Scalar: cout<<"NodeType:Scalar"<<endl; break;
   case YAML::NodeType::Sequence: cout<<"NodeType:Sequence"<<endl; break;
   case YAML::NodeType::Map: cout<<"NodeType:Map"<<endl; break;//输出为Map
   case YAML::NodeType::Undefined: cout<<"NodeType:Undefined"<<endl; break;
  }
}

//=============================================================================
int main(int argc ,char **argv)
{
  test11(); 
  return 0;  
}
~~~

测试输出文件：

~~~yaml
#test1
systemLog:
  destination: file
  path: c:\data\log\mongod.log
storage:
  dbPath: c:\data\db
security:
  authorization: enabled
net:
  bindIp: 127.0.0.1
  port: 27017
  
#test2
username: glimix
password: 111222

#test4
0: glimix
1: 123
2: 3.1415926
char:
  - a
bool:
  - true
  
#test5
- 100
- -45
- 50

#test6
name: John Smith
age: 37
spouse:
  name: Jane Smith
  age: 25
children:
  - name: Jimmy Smith
    age: 15
  - name: Jenny Smith
    age: 12
  - "{name: Alex Smith, age: 14}"
  - "name: Alex Smith, age: 14"
  - {name: Alex Smith, age: 14}
  - {name: Alex Smith, age: 14}

#test7
&1
key: value
seq: &2
  - &3 "element #1"
  - second element
mirror: *3
self: *1
*3: *2

#test8
[2, 3, 5, 7, 11, 13]

#test9
{1B: Prince Fielder, 2B: Rickie Weeks, LF: Ryan Braun, RF: Corey Hart, C: Jonathan Lucroy}
~~~



site.yaml文件（*注意：冒号和其后内容之间有一空格*）：

~~~yaml
systemLog:
    destination: file
    path: c:\data\log\mongod.log
storage:
    dbPath: c:\data\db
security:
     authorization: enabled
net:
    bindIp: 127.0.0.1
    port: 27017
~~~

CMakeLists.txt文件：

~~~cmake
# 使用c++11编译
add_compile_options(-std=c++11)
cmake_minimum_required(VERSION 2.6)
project(test)

find_package(yaml-cpp REQUIRED)

Message("YAML_INCLUDE:${YAML_CPP_INCLUDE_DIR}")

include_directories(${PROJECT_SOURCE_DIR} ${YAML_CPP_INCLUDE_DIR})
set(LIBS ${YAML_CPP_LIBRARIES})
add_executable(test main.cpp)
target_link_libraries(test ${LIBS})
~~~

## 结合ROS+OpenCV的使用

在许多ROS项目中使用YAML文件作为配置文件可以简化程序、提高代码的可读性和可修改性，在结合OpenCV的一些API，可以很方便地对配置信息进行修改，而不需要修改程序。下面是结合ROS+OpenCV读取YAML文件的一小段示例代码（未运行）。

~~~c++
#include <ros/ros.h>
#include <ros/package.h> 		//ros::packgae
#include <opencv2/opencv.hpp> 	//cv::FileStorage

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_name");
    string packagePath = ros::package::getPath("package_name");		//使用ROS包名读取包的地址
    string configPath = packagePath + "//config//config_name.yaml";	//YAML文件
    cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);	//cv::FileStorage适用于XML/YAML/JSON文件
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }
    string config_variable_name = fsSettings["config_variable_name"];
    //...
	return 0;
}
~~~

YAML文件：

~~~yaml
config_variable_name: value
str_name: "value"
...
~~~

## 参考资料

https://github.com/jbeder/yaml-cpp/wiki/Tutorial

http://www.glimix.com/archives/1570

http://yaml.org/

