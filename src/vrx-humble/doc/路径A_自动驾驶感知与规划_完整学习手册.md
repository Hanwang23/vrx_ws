# 🔥 路径A：自动驾驶感知与规划 — 完整学习手册

> **适用对象**：985船舶与海洋工程硕士（海洋智能与无人技术方向），本科双非机器人工程，2029年毕业
> **考研专业课**：自动控制原理 + 现代控制理论（有扎实的控制理论基础）
> **导师合作方**：云洲智能（无人船头部）+ 字节跳动
> **核心目标**：用海洋项目练手，用自动驾驶通用技术栈打底，毕业跳脱海洋行业
> **生成日期**：2026-07-09

---

## 📖 文档目录

| 模块 | 内容 | 字数 |
|------|------|------|
| 模块一 | C++编程 + Python进阶 + Linux + Git + 数学基础 | ~8000字 |
| 模块二 | 深度学习 + 目标检测(2D) + 分割 + 跟踪 + 关键点检测 | ~8300字 |
| 模块三 | 视觉SLAM + LiDAR SLAM + 惯性导航 + 组合导航 + BEV定位 | ~8000字 |
| 模块四 | 点云处理 + 3D检测(Point/Voxel/BEV) + 占据预测 + 数据集 | ~8300字 |
| 模块五 | 全局/局部规划 + 运动规划 + MPC + 行为决策 + Nav2 + 控制理论应用 | ~7900字 |
| 模块六 | ROS2 + 仿真环境 + 模型部署 + Apollo/Autoware + 项目实战 + 求职指南 | ~9500字 |

> **总计约5万字**，覆盖自动驾驶感知与规划方向的完整技术栈。

---

# 模块一：编程基础与数学基础

## Part 1: C++ 编程（自动驾驶必备）

### 1.1 C++11/14/17 核心特性

#### 1.1.1 智能指针（shared_ptr, unique_ptr, weak_ptr）

**学什么**：掌握 `std::unique_ptr`（独占所有权）、`std::shared_ptr`（共享所有权，引用计数）、`std::weak_ptr`（弱引用，打破循环引用）。理解三种智能指针的使用场景、自定义删除器（custom deleter）、`std::make_unique` 和 `std::make_shared` 的区别。了解引用计数的线程安全性问题。

**为什么学**：自动驾驶系统代码中，大量模块之间共享传感器数据（如点云、图像帧）。智能指针是管理这些共享资源的标准方式。Apollo、Autoware 等框架中几乎所有模块接口都使用智能指针传递数据。手动管理 `new/delete` 在大型项目中极易出错，智能指针是现代 C++ 的基石。

**学到什么程度**：
- 能根据所有权语义正确选择智能指针类型
- 理解 `shared_ptr` 的引用计数机制和控制块（control block）结构
- 能用 `weak_ptr` 解决循环引用问题（这是面试高频题）
- 了解 `make_shared` 相比 `shared_ptr(new T)` 的内存分配优势（单次分配 vs 两次分配）
- 掌握自定义删除器的写法（例如管理 CUDA 内存、文件句柄等非堆资源）

**学习时间**：3-4 天

**推荐资源**：
- 书籍：《Effective Modern C++》（Scott Meyers）Item 18-22，这是最权威的智能指针讲解
- 书籍：《C++ Primer》（第5版）第 12 章动态内存
- 课程：Cherno 的 C++ 系列 YouTube 视频（B站有搬运，搜索"Cherno C++ 智能指针"）
- 博客：侯捷 C++11 新特性系列课程（B站搜索"侯捷 C++11"）
- 在线：cppreference.com 智能指针页面（最准确的参考）

**检验标准**：能手写一个简单的引用计数智能指针（面试题）；能在给定代码中识别循环引用并用 `weak_ptr` 修复。

---

#### 1.1.2 Lambda 表达式

**学什么**：Lambda 的完整语法 `[capture](params) -> return_type { body }`。捕获方式（值捕获、引用捕获、`this` 捕获、广义捕获 C++14）。`std::function` 和 `std::bind`。Lambda 的本质（编译器生成的匿名函数对象）。泛型 Lambda（C++14）。

**为什么学**：自动驾驶框架中大量使用回调函数（callback），如传感器数据到达回调、规划完成回调等，Lambda 是写回调最简洁的方式。Autoware.universe 中的 ROS2 回调、Apollo 中的 CyberRT 回调，都大量使用 Lambda。

**学到什么程度**：
- 熟练使用各种捕获方式，能解释 `[=]`、`[&]`、`[this]`、`[=, &x]` 的区别
- 理解 Lambda 捕获的生命周期问题（引用捕获悬垂引用的陷阱）
- 能用 Lambda 替代简单的函数对象和 `std::bind`
- 了解泛型 Lambda `[](auto x) { return x * 2; }` 的用法

**学习时间**：2 天

**推荐资源**：
- 书籍：《Effective Modern C++》Item 31-34（避免默认捕获陷阱）
- 书籍：《C++ Primer》第 10.3.2 节
- 视频：The Cherno "Lambda Expressions in C++"（B站搬运）
- 博客：https://learnmoderncpp.com/ 系列教程

**检验标准**：能将 `std::bind` 代码重写为 Lambda 形式；能正确解释 `[&]` 捕获在异步场景下的生命周期问题。

---

#### 1.1.3 auto 类型推导

**学什么**：`auto` 的类型推导规则（模板参数推导规则）、`decltype`、`decltype(auto)`。`auto` 在迭代器、Lambda、复杂类型声明中的简化作用。`auto` 的限制（不能用于函数参数、类成员等 C++11 限制，C++20 才支持函数参数 auto）。

**为什么学**：自动驾驶代码中模板和迭代器用得非常多，`auto` 能大幅减少代码冗余，提高可读性。例如 `for (auto& point : point_cloud)` 这种写法在感知模块中随处可见。

**学到什么程度**：能在日常编码中合理使用 `auto`，理解 `auto` 推导可能带来的引用丢失问题（`auto` 会去掉引用），知道何时应该显式写类型。

**学习时间**：1 天

**推荐资源**：
- 书籍：《Effective Modern C++》Item 2-6
- cppreference.com auto 词条

**检验标准**：能正确推导 `const int& x = ...; auto y = x;` 中 `y` 的类型。

---

#### 1.1.4 移动语义与右值引用

**学什么**：左值/右值/将亡值（xvalue）的概念。右值引用 `T&&`。移动构造函数和移动赋值运算符。`std::move` 的本质（类型转换，不移动任何东西）。完美转发 `std::forward`。Copy Elision 和 RVO/NRVO。

**为什么学**：自动驾驶系统对性能要求极高，点云数据动辄数十万点，图像帧几 MB，频繁拷贝代价巨大。移动语义可以避免不必要的深拷贝。理解移动语义才能读懂框架源码中的 `std::move` 和 `&&` 重载。

**学到什么程度**：
- 能手写一个支持移动语义的类（Rule of Five）
- 理解 `std::move` 和 `std::forward` 的区别
- 知道何时编译器会自动使用移动语义（RVO），何时需要显式 `std::move`
- 理解万能引用（universal reference / forwarding reference）`T&&` 在模板中的含义

**学习时间**：4-5 天（这是 C++ 中较难的部分，需要反复理解）

**推荐资源**：
- 书籍：《Effective Modern C++》Item 23-30（最精华的部分）
- 书籍：《深入理解 C++ 对象模型》补充理解
- 视频：CppCon 2019 "Back to Basics: Move Semantics"（YouTube，B站搜索搬运）
- 视频：侯捷 C++11 移动语义（B站）
- 博客：https://www.cprogramming.com/c++11/rvalue-references-and-move-semantics-in-c++11.html

**检验标准**：能画出一个自定义类在拷贝/移动场景下的资源所有权转移图；能解释 `std::forward` 在完美转发中的作用。

---

#### 1.1.5 结构化绑定（C++17）

**学什么**：`auto [x, y] = pair;`、`auto& [key, value] = *map_iter;`。支持结构体、`pair`、`tuple`、数组的解构。绑定规则（变量个数必须匹配成员数）。

**为什么学**：遍历 map 时 `for (auto& [key, value] : my_map)` 比 `iter->first`、`iter->second` 清晰得多。自动驾驶代码中地图数据、配置参数等大量使用 map，结构化绑定让代码可读性大幅提升。

**学到什么程度**：能在日常编码中熟练使用，理解其限制。

**学习时间**：0.5 天

**推荐资源**：
- cppreference.com "Structured bindings" 词条
- 书籍：《C++17 - The Complete Guide》（Nicolai Josuttis）

**检验标准**：能用结构化绑定重写一段使用 `iter->first/second` 的代码。

---

#### 1.1.6 std::optional 和 std::variant（C++17）

**学什么**：`std::optional<T>` 的用法（表示可能有值也可能没有值，替代返回特殊值如 -1、nullptr）。`std::variant<Args...>` 的用法（类型安全的 union）。`std::visit` 访问 variant。`std::monostate`（表示空状态）。

**为什么学**：自动驾驶模块中很多函数可能失败（如检测到目标但无法跟踪），用 `optional` 表示比用哨兵值更安全、更清晰。`variant` 可用于表示多态传感器数据（Camera、Lidar、Radar 数据的统一容器），比继承体系更轻量。

**学到什么程度**：
- 能用 `optional` 替代返回 `nullptr` 或特殊值的函数
- 能用 `variant` + `std::visit` 实现访问者模式
- 理解 `optional` 与指针的区别（值语义 vs 引用语义）

**学习时间**：1-2 天

**推荐资源**：
- cppreference.com
- 书籍：《C++17 - The Complete Guide》
- 博客：https://www.cppstories.com/2018/06/optional-modern-cpp/（Bartlomiej Filipek 的博客系列质量很高）

**检验标准**：能设计一个函数，返回 `std::optional<DetectionResult>` 来表示检测可能失败的情况。

---

### 1.2 STL 容器深入

#### 1.2.1 vector 原理

**学什么**：`std::vector` 的内存布局（连续内存）。扩容策略（通常 2x 或 1.5x 增长）。`push_back` 的摊还时间复杂度。`reserve` vs `resize` 的区别。`emplace_back` vs `push_back` 的区别（原地构造 vs 拷贝/移动构造）。迭代器失效规则。

**为什么学**：`vector` 是自动驾驶代码中使用最频繁的容器，存储点云、轨迹点、路径等。理解其内部原理才能写出高性能代码。例如，预先 `reserve` 可以避免频繁扩容；理解迭代器失效可以避免崩溃 bug。

**学到什么程度**：
- 能解释 `vector` 的内存布局和扩容机制
- 知道 `reserve` 不改变 `size` 只改变 `capacity`
- 理解 `push_back` 和 `emplace_back` 的区别，优先使用 `emplace_back`
- 掌握迭代器失效规则（插入、删除操作后哪些迭代器会失效）
- 知道 `vector<bool>` 是特化版本，行为不同于普通 vector

**学习时间**：2 天

**推荐资源**：
- 书籍：《STL 源码剖析》（侯捷）第 4 章 vector
- 视频：CppCon 2019 "Back to Basics: Standard Library Containers"
- 书籍：《The C++ Standard Library》（Nicolai Josuttis）
- 在线：https://www.bfilipek.com/2020/04/vector-story.html

**检验标准**：能手写一个简化版 vector（支持 push_back、自动扩容、RAII）；能解释 vector 扩容的摊还分析。

---

#### 1.2.2 map vs unordered_map

**学什么**：`std::map`（红黑树，有序，O(log n) 查找）vs `std::unordered_map`（哈希表，无序，O(1) 平均查找）。自定义哈希函数。哈希冲突解决（链地址法 vs 开放地址法）。负载因子（load factor）和 rehash。迭代器稳定性。

**为什么学**：自动驾驶中经常需要快速查找，如通过 ID 查找跟踪目标（`map<track_id, Track>`）、车道线拓扑关系等。选择合适的容器直接影响系统性能。unordered_map 在大数据量查找时通常更快，但需要理解其哈希机制。

**学到什么程度**：
- 能根据使用场景选择 map 或 unordered_map
- 能为自定义类型编写哈希函数
- 知道 unordered_map 在最坏情况下的退化问题（哈希冲突）
- 了解 `std::flat_map`（C++23）作为替代方案

**学习时间**：2 天

**推荐资源**：
- 书籍：《STL 源码剖析》红黑树和哈希表章节
- 书籍：《Effective STL》Item 21-25
- 视频：CppCon "Designing a Fast, Efficient, Cache-Friendly Hash Table"
- 在线：https://en.cppreference.com/w/cpp/container

**检验标准**：能为一个 `struct Point { double x, y, z; }` 编写自定义哈希函数和相等比较函数。

---

#### 1.2.3 priority_queue 和 deque

**学什么**：`std::priority_queue`（基于堆，默认最大堆）的接口和自定义比较器。`std::deque`（双端队列）的内存结构（分段连续）。`priority_queue` 的底层实现（`make_heap`/`push_heap`/`pop_heap`）。

**为什么学**：自动驾驶中 A* 搜索、路径规划的 open list 通常用 priority_queue 实现。规划模块中按代价排序的候选路径管理也用优先队列。理解 deque 有助于理解 STL 中 stack 和 queue 的默认底层容器。

**学到什么程度**：
- 能用 `priority_queue` 实现 A* 算法的 open list
- 能自定义比较器（lambda 或仿函数）实现最小堆
- 知道 deque 的分段连续存储结构
- 知道 deque 不是完全连续的，不适合需要连续内存的场景

**学习时间**：1-2 天

**推荐资源**：
- 书籍：《STL 源码剖析》第 5 章
- 书籍：《算法导论》堆排序章节（理解堆的本质）
- cppreference.com priority_queue 词条

**检验标准**：能用 `priority_queue` 实现 Dijkstra 算法。

---

### 1.3 内存管理

#### 1.3.1 堆与栈

**学什么**：栈内存（自动管理、速度快、空间有限、LIFO）和堆内存（手动管理、较慢、空间大）的区别。栈溢出（stack overflow）的常见原因。内存布局（text/data/bss/heap/stack）。`alloca` vs `malloc`。

**为什么学**：自动驾驶系统中，点云数据（几十 MB）不能放栈上，必须用堆或内存池。理解堆栈区别才能排查段错误（segfault）和栈溢出。

**学到什么程度**：能画出程序的内存布局图，能解释栈溢出和堆溢出的后果。

**学习时间**：1 天

**推荐资源**：
- 书籍：《深入理解计算机系统》（CSAPP）第 9 章
- 视频：CSAPP 配套视频（B站搜索"CSAPP 内存"）

**检验标准**：能解释为什么递归过深会导致栈溢出。

---

#### 1.3.2 RAII（Resource Acquisition Is Initialization）

**学什么**：RAII 的核心思想——将资源生命周期绑定到对象生命周期。构造函数获取资源、析构函数释放资源。RAII 在智能指针、文件流、锁守卫（`std::lock_guard`）中的应用。

**为什么学**：RAII 是 C++ 最重要的编程范式之一。自动驾驶框架中的资源管理（GPU 内存、传感器连接、文件句柄）都依赖 RAII。Apollo 和 Autoware 的代码中处处体现 RAII 思想。

**学到什么程度**：能用 RAII 思想封装任何资源（文件、锁、GPU 内存等）。

**学习时间**：1 天

**推荐资源**：
- 书籍：《Effective C++》Item 13（以资源管理对象管理资源）
- 书籍：《C++ Primer》第 12 章
- 视频：Cherno "RAII in C++"

**检验标准**：能实现一个 RAII 风格的文件操作类。

---

#### 1.3.3 内存泄漏排查

**学什么**：常见内存泄漏场景（循环引用、未 delete、异常导致未释放）。排查工具：`valgrind`（`valgrind --leak-check=full ./program`）、`AddressSanitizer`（`-fsanitize=address`）、`LeakSanitizer`。智能指针如何预防泄漏。

**为什么学**：自动驾驶系统通常是长时间运行的程序（7x24），内存泄漏会导致系统逐渐变慢甚至崩溃。掌握排查工具是工程必备技能。

**学到什么程度**：
- 能用 Valgrind 检测内存泄漏和非法访问
- 能用 AddressSanitizer（ASan）在编译时检测内存问题
- 了解 LeakSanitizer 是 ASan 的子集
- 能在 CMake 中正确配置 Sanitizer

**学习时间**：1-2 天

**推荐资源**：
- Valgrind 官方文档：https://valgrind.org/docs/manual/
- Google Sanitizer 文档：https://github.com/google/sanitizers
- 视频：CppCon "Sanitizers: Getting Your Code Healthy"
- 博客：https://clang.llvm.org/docs/AddressSanitizer.html

**检验标准**：能用 Valgrind 或 ASan 找出一段有内存泄漏的代码中的问题并修复。

---

### 1.4 多线程编程

#### 1.4.1 std::thread 与基本线程管理

**学什么**：`std::thread` 的创建、`join`、`detach`。线程参数传递（值传递、引用传递需要 `std::ref`）。`std::this_thread` 命名空间。线程 ID。RAII 线程管理类（类似 `std::jthread` C++20）。

**为什么学**：自动驾驶系统是典型的多线程程序——感知线程、定位线程、规划线程、控制线程并行运行。理解线程管理是基础。

**学到什么程度**：能创建和管理多个线程，理解 `join` 和 `detach` 的区别，知道线程函数的参数传递规则。

**学习时间**：2 天

**推荐资源**：
- 书籍：《C++ Concurrency in Action》（Anthony Williams）——多线程编程的圣经
- 视频：Cherno "Multithreading in C++"（B站搬运）
- cppreference.com std::thread 词条

**检验标准**：能用多线程实现生产者-消费者模型的基本版本。

---

#### 1.4.2 mutex、lock_guard、unique_lock

**学什么**：`std::mutex`、`std::recursive_mutex`。`std::lock_guard`（RAII 锁管理）。`std::unique_lock`（灵活锁管理，支持延迟加锁、提前解锁）。死锁（四个必要条件）。`std::lock` 同时锁多个 mutex（避免死锁）。

**为什么学**：自动驾驶中多个线程共享数据结构（如共享地图、共享目标列表），必须用 mutex 保护。Apollo 的数据通道（Channel）内部就使用了互斥锁。锁使用不当会导致死锁，系统卡死。

**学到什么程度**：
- 能用 `lock_guard` 和 `unique_lock` 正确保护共享数据
- 理解死锁产生的条件，能识别和避免死锁
- 知道 RAII 锁管理的优势
- 了解 `std::scoped_lock`（C++17，同时锁多个 mutex）

**学习时间**：2 天

**推荐资源**：
- 书籍：《C++ Concurrency in Action》第 3-4 章
- 视频：CppCon 2017 "Deciphering C++ Threading Models"
- 书籍：《Effective Modern C++》Item 37-40

**检验标准**：能正确编写一个多线程安全的共享数据结构，无死锁风险。

---

#### 1.4.3 condition_variable

**学什么**：`std::condition_variable` 的 `wait`、`notify_one`、`notify_all`。条件变量的典型使用模式（检查条件 -> 加锁 -> wait -> 处理）。虚假唤醒（spurious wakeup）及 `wait` 的谓词版本。`std::condition_variable_any`。

**为什么学**：自动驾驶系统中，数据驱动的处理流程（传感器数据到达 -> 触发处理）大量使用条件变量。ROS2/Apollo 的回调队列底层也是类似机制。

**学到什么程度**：能用条件变量实现完整的生产者-消费者模型，理解虚假唤醒。

**学习时间**：2 天

**推荐资源**：
- 书籍：《C++ Concurrency in Action》第 4 章
- cppreference.com

**检验标准**：能手写一个线程安全的阻塞队列（BlockingQueue）。

---

#### 1.4.4 std::atomic

**学什么**：`std::atomic<T>` 的基本用法。原子操作（`load`、`store`、`fetch_add`、`compare_exchange_weak/strong`）。内存顺序（`memory_order_relaxed`、`acquire`、`release`、`seq_cst`）。`volatile` 与 `atomic` 的区别。

**为什么学**：简单的标志位（如 `is_running_`）用 atomic 比 mutex 更高效。自动驾驶系统中的性能敏感路径常用 atomic。理解 CAS 操作是理解无锁编程的基础。

**学到什么程度**：
- 能用 `std::atomic<bool>` 做线程间标志位通信
- 理解 `memory_order_seq_cst` 是默认也是最安全的
- 了解基本的 CAS（Compare-And-Swap）思想
- 不需要精通所有 memory order，但要知道存在这些选项

**学习时间**：2-3 天

**推荐资源**：
- 书籍：《C++ Concurrency in Action》第 5-7 章
- 视频：CppCon 2017 "Atomic Weapons"（经典讲内存模型的）
- 博客：https://preshing.com/20120612/an-introduction-to-lock-free-programming/

**检验标准**：能用 `atomic<bool>` 实现一个线程安全的自旋锁。

---

#### 1.4.5 future/promise 和 async

**学什么**：`std::future`、`std::promise`、`std::packaged_task`。`std::async` 的启动策略（`launch::async` vs `launch::deferred`）。`future::get()` 阻塞等待结果。`std::shared_future`（多个等待者）。

**为什么学**：异步任务处理在自动驾驶中很常见——异步加载地图、异步发送日志、异步计算预测等。future/promise 是获取异步结果的标准方式。

**学到什么程度**：能用 `std::async` 执行异步任务并获取结果，理解 `get()` 只能调用一次。

**学习时间**：1-2 天

**推荐资源**：
- 书籍：《C++ Concurrency in Action》第 4 章
- cppreference.com

**检验标准**：能用 `std::async` 并行化一个串行计算任务。

---

#### 1.4.6 线程池

**学什么**：线程池的基本原理（固定数量的工作线程 + 任务队列）。实现一个简单的线程池（任务队列 + 条件变量 + 工作线程循环取任务）。C++23 `std::execution` 简介。

**为什么学**：自动驾驶系统中，对每一帧传感器数据的处理可能需要并行执行多个任务，创建/销毁线程的开销太大，线程池是标准解决方案。Apollo 的 CyberRT Scheduler 底层就使用了线程池。

**学到什么程度**：能手写一个支持 `enqueue` 任务的简单线程池（100-150 行代码）。

**学习时间**：2-3 天

**推荐资源**：
- GitHub：https://github.com/progschj/ThreadPool （经典简易线程池实现，Star 7k+）
- 书籍：《C++ Concurrency in Action》
- 博客：https://github.com/mtrebi/thread-pool （带详细注释的实现）
- 视频：YouTube "C++ Thread Pool Tutorial"（搜索 "C++ thread pool implementation"）

**检验标准**：能从零实现一个线程池，支持提交任务并获取 future 结果。

---

### 1.5 模板编程基础

#### 1.5.1 函数模板与类模板

**学什么**：函数模板的定义和使用、模板参数推导。类模板的定义和使用。模板特化（全特化和偏特化）。模板参数默认值。`typename` vs `class` 的区别。非类型模板参数。

**为什么学**：自动驾驶框架大量使用模板实现泛型编程。例如传感器数据容器模板 `SensorData<T>`、状态估计器模板等。不理解模板就无法阅读框架源码。

**学到什么程度**：
- 能编写函数模板和类模板
- 理解全特化和偏特化的语法
- 能读懂 STL 源码中的模板代码
- 不需要掌握模板元编程（template metaprogramming）的高级技巧

**学习时间**：3 天

**推荐资源**：
- 书籍：《C++ Primer》第 16 章
- 书籍：《C++ Templates: The Complete Guide》（入门级阅读前几章即可）
- 视频：CppCon 2019 "Back to Basics: Templates"（B站有搬运）

**检验标准**：能实现一个简单的 `Array<T, N>` 模板类。

---

#### 1.5.2 SFINAE 基础

**学什么**：SFINAE（Substitution Failure Is Not An Error）的基本概念。`std::enable_if`。`std::is_same`、`std::is_arithmetic` 等类型萃取（type traits）。C++17 `if constexpr` 作为更简洁的替代。`std::void_t`。

**为什么学**：阅读自动驾驶框架源码时，经常会看到 `enable_if` 和类型萃取，不理解 SFINAE 会看不懂这些代码。C++17 的 `if constexpr` 大幅简化了条件编译逻辑。

**学到什么程度**：
- 理解 SFINAE 的基本原理
- 能用 `if constexpr` 替代大部分 SFINAE 场景
- 了解常用 type traits 的作用
- 不需要精通复杂的 SFINAE 技巧

**学习时间**：2 天

**推荐资源**：
- 书籍：《Effective Modern C++》Item 26-30
- 视频：CppCon "Back to Basics: Templates" 和 "SFINAE"
- 博客：https://www.fluentcpp.com/2018/05/15/make-sfinae-pretty-1-what-value-sfinae-brings-to-code/

**检验标准**：能用 `if constexpr` 编写一个函数，对不同类型参数有不同的处理逻辑。

---

### 1.6 CMake 构建系统

**学什么**：CMake 基本语法（`cmake_minimum_required`、`project`、`add_executable`、`add_library`、`target_link_libraries`）。`CMakeLists.txt` 结构。`find_package` 查找第三方库。现代 CMake 最佳实践（target-based，`target_include_directories` 而非 `include_directories`）。编译选项设置（`target_compile_options`）。安装规则（`install`）。`cmake -DCMAKE_BUILD_TYPE=Release` 构建类型。

**为什么学**：CMake 是 C++ 项目的事实标准构建系统。Apollo、Autoware、ROS2 都使用 CMake。不会 CMake 就无法参与任何 C++ 开源自动驾驶项目。

**学到什么程度**：
- 能为一个中等规模的 C++ 项目编写 CMakeLists.txt
- 能正确链接第三方库（OpenCV、PCL、Eigen 等）
- 理解现代 CMake 的 target-based 思想
- 能设置 Debug/Release 构建

**学习时间**：3-4 天

**推荐资源**：
- 书籍/教程：https://cliutils.gitlab.io/modern-cmake/（Modern CMake 在线教程，极力推荐）
- 视频：Jason Turner "C++ Weekly" 系列中 CMake 相关集
- GitHub：https://github.com/pr0g/cmake-examples（CMake 示例项目集合）
- 官方文档：https://cmake.org/cmake/help/latest/
- 博客：https://pabloariasal.github.io/2018/02/19/its-time-to-do-cmake-right/

**检验标准**：能从零创建一个 C++ 项目，包含多个库和可执行文件，正确配置依赖关系，用 CMake 构建成功。

---

### 1.7 设计模式

#### 1.7.1 单例模式

**学什么**：单例模式的实现（C++11 静态局部变量是线程安全的）。饿汉式 vs 懒汉式。`std::call_once` 实现。单例的缺点（全局状态、测试困难）。

**为什么学**：自动驾驶中的日志系统、配置管理器、全局状态机通常用单例实现。

**学到什么程度**：能用 C++11 的 Meyers' Singleton 实现线程安全的单例。

**学习时间**：0.5 天

**推荐资源**：
- 书籍：《设计模式》（GoF）
- 书籍：《Effective C++》Item 4
- 视频：The Cherno "Singleton Design Pattern in C++"

**检验标准**：能实现线程安全的懒汉单例。

---

#### 1.7.2 工厂模式

**学什么**：简单工厂、工厂方法、抽象工厂。用 `std::map<string, Creator>` 注册和创建对象。`std::variant` 作为工厂的替代方案。

**为什么学**：自动驾驶中，根据配置创建不同的传感器驱动（Camera、Lidar、Radar）、不同的规划算法（Lattice、EM Planner）等场景大量使用工厂模式。

**学到什么程度**：能用工厂模式实现可扩展的算法插件机制。

**学习时间**：1 天

**推荐资源**：
- 书籍：《大话设计模式》（入门友好）
- 视频：Cherno "Factory Pattern"

**检验标准**：能实现一个规划算法工厂，根据配置字符串创建不同的规划器。

---

#### 1.7.3 观察者模式

**学什么**：观察者模式的结构（Subject/Observer）。发布-订阅机制。回调函数注册和通知。C++ 实现方式（`std::function` + `std::vector` 存储回调）。ROS2 的话题通信本质上就是观察者模式的分布式版本。

**为什么学**：自动驾驶系统是事件驱动的——传感器数据到达事件、障碍物检测事件、规划完成事件等。观察者模式是实现模块间通信的核心模式。

**学到什么程度**：能实现一个支持多类型事件的观察者模式框架。

**学习时间**：1 天

**推荐资源**：
- 书籍：《设计模式》（GoF）
- 书籍：《大话设计模式》
- GitHub：搜索 "C++ observer pattern implementation"

**检验标准**：能实现一个简单的事件发布-订阅系统。

---

#### 1.7.4 策略模式

**学什么**：策略模式的结构（Context/Strategy 接口/具体策略）。运行时切换算法。与 `std::function` 和 Lambda 的结合使用。

**为什么学**：自动驾驶中，路径规划有多种策略（A*、RRT、Lattice），跟踪算法有多种策略（MPC、PID、Stanley），策略模式使得算法可插拔、可切换。

**学到什么程度**：能用策略模式实现算法插件化。

**学习时间**：0.5 天

**推荐资源**：
- 书籍：《设计模式》（GoF）
- 视频：The Cherno "Strategy Pattern"

**检验标准**：能用策略模式实现一个控制算法选择器。

---

### 1.8 LeetCode 刷题策略

**为什么学**：刷题是为了训练算法思维、数据结构运用能力和代码编写速度。自动驾驶公司的面试（百度、小马智行、华为、Momenta 等）都会考算法题。

**推荐题单**（按优先级排序）：

**入门必刷（前 100 题精选）**：
- 数组/字符串：两数之和(#1)、三数之和(#15)、盛最多水的容器(#11)、接雨水(#42)
- 链表：反转链表(#206)、合并两个有序链表(#21)、环形链表(#141)、LRU缓存(#146)
- 树：二叉树的层序遍历(#102)、验证二叉搜索树(#98)、二叉树的最近公共祖先(#236)
- 排序/搜索：二分查找(#704)、搜索旋转排序数组(#33)、快速排序手写
- 动态规划：爬楼梯(#70)、最长递增子序列(#300)、零钱兑换(#322)、编辑距离(#723)
- 图：岛屿数量(#200)、课程表(#207)、拓扑排序

**进阶专题（自动驾驶相关重点）**：
- BFS/DFS：#127 单词接龙、#130 被围绕的区域
- 优先队列/堆：#23 合并K个升序链表、#295 数据流的中位数
- 前缀和：#560 和为K的子数组
- 滑动窗口：#76 最小覆盖子串、#239 滑动窗口最大值
- 并查集：#547 省份数量
- 单调栈：#84 柱状图中最大的矩形

**刷题策略**：
1. 第一轮（2 个月）：按分类刷，每个分类 10-15 题，重在理解套路
2. 第二轮（1 个月）：混合刷，提升解题速度
3. 面试前（2 周）：只刷高频题和自己做错的题

**学习时间**：持续 3-4 个月，每天 1-2 题

**推荐资源**：
- 书籍：《剑指 Offer》（何海涛）——面试必备
- 书籍：《代码随想录》（programmercarl.com）——非常适合系统刷题
- 网站：LeetCode（https://leetcode.cn/）
- GitHub：https://github.com/youngyangyang04/leetcode-master（代码随想录配套）
- 视频：代码随想录 B站视频系列
- 题单：LeetCode Hot 100（https://leetcode.cn/studyplan/top-100-liked/）

**检验标准**：能在 20 分钟内完成一道中等难度题；Hot 100 至少完成 80%。

---

## Part 2: Python 进阶

### 2.1 NumPy 完全掌握

#### 2.1.1 广播机制（Broadcasting）

**学什么**：广播规则（从右向左比较维度，维度相等或其中一个为 1 则可广播）。常见广播场景（矩阵 + 向量、矩阵 + 标量）。广播的内存效率（不实际复制数据）。

**为什么学**：自动驾驶中的数据处理（点云坐标变换、图像预处理、特征归一化）大量使用广播来避免显式循环。例如将所有点云点减去均值，一个广播操作即可完成。

**学到什么程度**：能熟练使用广播进行矩阵/向量运算，遇到维度不匹配时能快速定位问题。

**学习时间**：1 天

**推荐资源**：
- 官方文档：https://numpy.org/doc/stable/user/basics.broadcasting.html
- 书籍：《Python Data Science Handbook》（Jake VanderPlas）第 2 章
- 视频：3Blue1Brown "Essence of Linear Algebra" 系列辅助理解

**检验标准**：能不查文档写出 `(3,1) + (1,4) -> (3,4)` 的广播结果。

---

#### 2.1.2 高级索引

**学什么**：花式索引（fancy indexing，用整数数组索引）。布尔索引（boolean indexing）。混合索引（切片 + 整数 + `np.newaxis`）。`np.where`、`np.argwhere`、`np.nonzero`。

**为什么学**：自动驾驶数据处理中经常需要筛选满足条件的点（如距离范围内的点云、置信度大于阈值的检测框），这些操作都依赖 NumPy 的高级索引。

**学到什么程度**：能用一行 NumPy 代码完成复杂的条件筛选和数据提取。

**学习时间**：1-2 天

**推荐资源**：
- 官方文档：https://numpy.org/doc/stable/reference/arrays.indexing.html
- 书籍：《Python Data Science Handbook》

**检验标准**：能用布尔索引从点云数组中提取指定范围内的点。

---

#### 2.1.3 向量化编程

**学什么**：用 NumPy 向量操作替代 Python for 循环。`np.vectorize`（注意它不真正加速，只是语法糖）。通用函数（ufunc）的概念。`np.apply_along_axis` 与向量化的对比。性能对比（向量化 vs 循环，通常快 100 倍以上）。

**为什么学**：自动驾驶数据量大（每秒几十帧点云，每帧数万点），Python 循环太慢，向量化是 Python 数据处理的性能基石。

**学到什么程度**：看到 for 循环处理 NumPy 数组时，能自动想到向量化替代方案。

**学习时间**：1 天

**推荐资源**：
- 书籍：《High Performance Python》（O'Reilly）
- 博客：https://numpy.org/doc/stable/reference/ufuncs.html
- 视频：Scipy Conference talks on vectorization

**检验标准**：能将一个包含 for 循环的点云处理函数重写为纯向量化版本，速度提升一个数量级以上。

---

#### 2.1.4 einsum（爱因斯坦求和）

**学什么**：`np.einsum` 的记法和用法（`ij,jk->ik` 表示矩阵乘法）。常见操作的 einsum 表示（矩阵乘、迹、转置、外积、batch 矩阵乘）。`np.einsum_path`（自动优化求和顺序）。

**为什么学**：自动驾驶中涉及大量多维数组运算（投影矩阵、坐标变换、注意力机制），einsum 用一种统一的语法表达各种张量运算，代码更清晰、更不容易出错。

**学到什么程度**：能用 `einsum` 表达常见的矩阵运算，理解下标含义。不需要精通所有用法，但遇到时能读懂。

**学习时间**：1-2 天

**推荐资源**：
- 官方文档：https://numpy.org/doc/stable/reference/generated/numpy.einsum.html
- 博客：https://rockt.github.io/2018/04/30/einsum（Tim Rocktäschel 的经典 einsum 教程）
- 视频：搜索 "einsum explained"

**检验标准**：能用 `einsum` 实现 batch 矩阵乘法和矩阵迹的计算。

---

### 2.2 Pandas 数据处理

**学什么**：`DataFrame` 和 `Series` 的基本操作。数据读写（`read_csv`、`read_parquet`、`to_csv`）。数据筛选（布尔索引、`query` 方法、`loc`/`iloc`）。分组聚合（`groupby`、`agg`、`transform`）。数据合并（`merge`、`concat`、`join`）。缺失值处理（`isnull`、`fillna`、`dropna`）。数据透视表（`pivot_table`）。

**为什么学**：自动驾驶开发中需要处理大量实验数据——传感器标定数据、A/B 测试结果、仿真评估指标、训练日志等。Pandas 是分析这些数据的主力工具。例如分析感知模块在不同场景下的精度指标（AP/mAP）。

**学到什么程度**：
- 能用 Pandas 读取、清洗、分析中等规模数据（百万行级）
- 能熟练使用 groupby 进行分组统计
- 能处理缺失值和异常值
- 熟悉 `apply` 但知道它很慢，优先使用向量化操作

**学习时间**：5-7 天

**推荐资源**：
- 书籍：《Python Data Science Handbook》（Jake VanderPlas）第 3 章
- 书籍：《利用 Python 进行数据分析》（Wes McKinney）
- 课程：Kaggle Learn "Pandas" 免费微课程（https://www.kaggle.com/learn/pandas）
- 官方文档：https://pandas.pydata.org/docs/
- 视频：Keith Galli "Pandas Tutorial"（B站有搬运）

**检验标准**：能用 Pandas 从原始数据表中生成一份自动驾驶场景测试报告（统计各场景类型通过率、平均误差等）。

---

### 2.3 可视化

#### 2.3.1 Matplotlib

**学什么**：基础绘图（`plot`、`scatter`、`bar`、`hist`）。子图布局（`subplots`、`GridSpec`）。图形美化（标题、标签、图例、颜色映射）。保存图片（`savefig`，DPI 设置）。3D 绘图（`plot_surface`、`scatter3D`）。动画（`FuncAnimation`，用于可视化轨迹）。

**为什么学**：自动驾驶开发中每天都在画图——损失曲线、规划轨迹、传感器数据、评估指标。Matplotlib 是最基础也最强大的可视化工具。

**学到什么程度**：能画出专业的论文级别图表，能可视化 3D 点云和轨迹。

**学习时间**：3-4 天

**推荐资源**：
- 书籍：《Python Data Science Handbook》第 4 章
- 官方教程：https://matplotlib.org/stable/tutorials/index.html
- GitHub：https://github.com/matplotlib/cheatsheets（Matplotlib 速查表）
- 视频：Corey Schafer "Matplotlib Tutorial"（B站搬运）
- 网站：https://matplotlib.org/stable/gallery/index.html（官方示例库，找到所需图形直接改）

**检验标准**：能画出一张包含多条轨迹对比的子图，有标题、图例、坐标轴标签，达到论文发表水平。

---

#### 2.3.2 Seaborn 和 Plotly

**学什么**：Seaborn 的统计可视化（`heatmap` 相关矩阵、`boxplot` 分布、`violinplot`、`pairplot` 散点矩阵）。Plotly 的交互式可视化（`plotly.express` 快速绑图、`plotly.graph_objects` 自定义、`Dash` 简介）。

**为什么学**：Seaborn 适合做论文中的统计图和混淆矩阵可视化。Plotly 适合做交互式数据探索和 Web 端可视化 demo（如展示规划结果的交互式动画）。

**学到什么程度**：能用 Seaborn 画出论文级别的统计图，能用 Plotly 做基本的交互式可视化。

**学习时间**：2-3 天（Seaborn 1 天，Plotly 1-2 天）

**推荐资源**：
- Seaborn 官方：https://seaborn.pydata.org/tutorial.html
- Plotly 官方：https://plotly.com/python/
- 视频：Data School "Seaborn Tutorial"（YouTube/B站）

**检验标准**：能用 Seaborn 画出一张高颜值的相关性热力图；能用 Plotly 做一个可旋转的 3D 点云可视化。

---

### 2.4 Python 高级特性

#### 2.4.1 装饰器（Decorator）

**学什么**：装饰器的本质（高阶函数，接收函数返回函数）。`@decorator` 语法。带参数的装饰器。类装饰器。`functools.wraps` 保留原函数信息。常用内置装饰器（`@staticmethod`、`@classmethod`、`@property`）。

**为什么学**：自动驾驶代码中装饰器常用于——性能计时（`@timer`）、日志记录（`@log_call`）、权限检查、结果缓存（`@lru_cache`）、重试机制（`@retry`）等。不理解装饰器就无法理解很多 Python 库代码。

**学到什么程度**：能自己编写装饰器（包括带参数的），理解装饰器的执行时机。

**学习时间**：1-2 天

**推荐资源**：
- 书籍：《流畅的 Python》（Luciano Ramalho）第 7 章和第 21 章
- 视频：Corey Schafer "Python Decorators"（B站搬运）
- 博客：https://realpython.com/primer-on-python-decorators/

**检验标准**：能编写一个带参数的 `@retry(max_attempts=3)` 装饰器。

---

#### 2.4.2 生成器（Generator）

**学什么**：生成器函数（`yield`）、生成器表达式（`(x**2 for x in range(10))`）。`yield from` 委托。惰性求值的优势。`itertools` 模块。

**为什么学**：处理大量传感器数据时（如逐帧读取数据集），生成器可以避免一次性加载全部数据到内存。自动驾驶数据集动辄几百 GB，生成器是必备技能。

**学到什么程度**：能用生成器实现惰性数据加载 pipeline。

**学习时间**：1 天

**推荐资源**：
- 书籍：《流畅的 Python》第 14 章（Iterables, Iterators, Generators）
- 视频：Corey Schafer "Generators"（B站搬运）
- 官方文档：https://docs.python.org/3/howto/functional.html#generators

**检验标准**：能用生成器实现一个逐帧读取 rosbag 的数据迭代器。

---

#### 2.4.3 上下文管理器（Context Manager）

**学什么**：`__enter__` 和 `__exit__` 协议。`contextlib.contextmanager` 装饰器简化写法。嵌套上下文管理器。

**为什么学**：自动驾驶中常见的上下文管理器场景——打开/关闭设备连接、计时器、临时修改配置、CUDA 流管理等。`with open(...)` 就是最常见的上下文管理器。

**学到什么程度**：能用两种方式实现上下文管理器（类和装饰器）。

**学习时间**：0.5 天

**推荐资源**：
- 书籍：《流畅的 Python》第 15 章
- 官方文档：https://docs.python.org/3/library/contextlib.html

**检验标准**：能编写一个 `@contextmanager` 风格的计时器。

---

#### 2.4.4 元类基础（Metaclass）

**学什么**：`type` 是元类。`__new__` vs `__init__`。`__metaclass__` 和 `metaclass=` 参数。`__class__` 属性。ABC（抽象基类）使用 `ABCMeta`。

**为什么学**：不需要精通元类编程，但需要理解它，因为很多框架（如 PyTorch 的 `nn.Module`、TensorFlow 的 Keras 层）内部使用了元类。理解元类有助于理解这些框架的设计。

**学到什么程度**：能读懂元类代码，理解 `type(name, bases, dict)` 的作用。不需要自己设计元类。

**学习时间**：1 天

**推荐资源**：
- 书籍：《流畅的 Python》第 21 章
- 视频：Corey Schafer "Metaprogramming"（B站搬运）
- 博客：https://realpython.com/python-metaclasses/

**检验标准**：能解释 `type(int)` 和 `type(type)` 的结果。

---

### 2.5 多进程/多线程/异步编程

#### 2.5.1 多线程（threading）

**学什么**：`threading.Thread`、`threading.Lock`、`threading.Event`、`threading.Semaphore`。GIL（全局解释器锁）的含义和影响——Python 多线程不能利用多核 CPU 执行计算密集型任务。

**为什么学**：理解 GIL 是理解 Python 并发编程的关键。自动驾驶中 Python 多线程主要用于 I/O 密集型任务（文件读写、网络请求、ROS 通信）。

**学习时间**：1 天

**推荐资源**：
- 书籍：《Python 并发编程原理》（Vishal Bodani）
- 视频：Corey Schafer "Threading"（B站搬运）

---

#### 2.5.2 多进程（multiprocessing）

**学什么**：`multiprocessing.Process`、`multiprocessing.Pool`。进程间通信（`Queue`、`Pipe`、`Shared Memory`）。`multiprocessing.Pool.map` 并行化。进程池 vs 线程池的选择。

**为什么学**：自动驾驶的 Python 数据处理 pipeline（数据预处理、批量推理、数据增强）通常用多进程并行化。例如用多进程并行处理多个 rosbag 文件。

**学习时间**：2 天

**推荐资源**：
- 官方文档：https://docs.python.org/3/library/multiprocessing.html
- 书籍：《High Performance Python》第 7 章
- 视频：Corey Schafer "Multiprocessing"（B站搬运）

**检验标准**：能用 `multiprocessing.Pool` 并行处理多个数据文件。

---

#### 2.5.3 异步编程（asyncio）

**学什么**：`async/await` 语法。事件循环（Event Loop）。`asyncio.gather` 并发执行。异步生成器。`aiohttp`（异步 HTTP 客户端）。

**为什么学**：自动驾驶中的数据上传/下载、与云端服务通信等 I/O 密集型任务适合用异步编程提高效率。

**学到什么程度**：能编写基本的异步程序，理解协程的调度方式。不需要精通。

**学习时间**：2 天

**推荐资源**：
- 书籍：《流畅的 Python》第 19-20 章
- 官方文档：https://docs.python.org/3/library/asyncio.html
- 视频：James Powell "Advanced asyncio"（PyCon 演讲）

**检验标准**：能用 `asyncio.gather` 并发发送多个 HTTP 请求。

---

### 2.6 类型注解与 dataclass

**学什么**：Python 类型注解语法（`def func(x: int) -> str:`）。`typing` 模块（`List`、`Dict`、`Optional`、`Union`、`Tuple`、`TypeVar`、`Generic`）。`dataclasses` 模块（`@dataclass`、`field`、`__post_init__`、`frozen=True`）。`mypy` 静态类型检查。Pydantic 简介（数据验证）。

**为什么学**：自动驾驶项目的 Python 代码量越来越大，类型注解能提高代码可维护性和 IDE 支持。`dataclass` 是定义数据结构的标准方式（替代普通 class + `__init__`），在自动驾驶的配置管理、数据模型定义中广泛使用。

**学到什么程度**：
- 能为函数和类添加完整的类型注解
- 能用 `dataclass` 定义配置类和数据结构
- 了解 `mypy` 的基本用法
- 了解 Pydantic 的基本用法（FastAPI 数据验证的基础）

**学习时间**：1-2 天

**推荐资源**：
- 书籍：《流畅的 Python》第 2 版 相关章节
- 官方文档：https://docs.python.org/3/library/dataclasses.html
- 官方文档：https://docs.python.org/3/library/typing.html
- mypy：https://mypy.readthedocs.io/
- Pydantic：https://docs.pydantic.dev/

**检验标准**：能用 `dataclass` 定义一个完整的自动驾驶配置类，并用类型注解使其通过 mypy 检查。

---

## Part 3: Linux 系统

### 3.1 基本命令与 Shell 脚本

**学什么**：
- 文件操作：`ls`、`cd`、`cp`、`mv`、`rm`、`mkdir`、`ln -s`（软链接）、`find`、`grep`、`wc`
- 权限管理：`chmod`、`chown`、`umask`
- 管道与重定向：`|`、`>`、`>>`、`2>&1`、`tee`
- 文本处理：`awk`、`sed`、`sort`、`uniq`、`cut`、`tr`
- 系统信息：`uname`、`df`、`du`、`free`
- Shell 脚本：变量、条件判断 `if []`、循环 `for/while`、函数、`$1` `$@` `$#` 参数、`set -e`（遇错停止）

**为什么学**：自动驾驶的开发、部署、调试全在 Linux 上完成。Apollo、Autoware 都是 Linux-only 的项目。Shell 脚本用于自动化数据处理、批量实验、部署流程等。

**学到什么程度**：
- 能熟练使用命令行进行日常开发操作
- 能用 `grep`/`find`/`awk`/`sed` 快速从日志文件中提取信息
- 能编写中等复杂度的 Shell 脚本（200 行以内）完成自动化任务
- 理解 `|` 管道和重定向的工作原理

**学习时间**：5-7 天

**推荐资源**：
- 书籍：《鸟哥的 Linux 私房菜》——中文 Linux 入门经典
- 书籍：《Linux Command Line and Shell Scripting Bible》
- 网站：https://www.linuxcommand.org/tlcl.php（The Linux Command Line，免费电子书）
- 视频：B站搜索"Linux 命令行教程"
- 练习：https://overthewire.org/wargames/bandit/（Bandit 游戏，边玩边学命令行）
- Shell 脚本：https://www.shellscript.sh/

**检验标准**：能写一个 Shell 脚本，批量处理 rosbag 文件并生成统计报告。

---

### 3.2 进程管理与内存管理

**学什么**：
- 进程管理：`ps aux`、`top`、`htop`、`kill`/`kill -9`、`nohup`、`nice`/`renice`、`&` 后台运行、`jobs`/`fg`/`bg`、`strace`（跟踪系统调用）
- 内存管理：`free -h`、`/proc/meminfo`、虚拟内存概念、OOM Killer、`mmap` 基本概念
- 进程间通信（IPC）基础：管道（pipe）、共享内存、消息队列、信号（signal）

**为什么学**：自动驾驶系统运行时需要监控各进程的 CPU/内存使用情况。当系统变慢时，需要定位是哪个进程/线程的瓶颈。理解 IPC 有助于理解 ROS2/Apollo 的通信机制。

**学到什么程度**：
- 能用 `top`/`htop` 监控系统资源
- 能用 `ps` 找到特定进程并管理
- 理解虚拟内存和物理内存的区别
- 了解 IPC 的几种基本方式
- 能用 `strace` 跟踪程序的系统调用

**学习时间**：2-3 天

**推荐资源**：
- 书籍：《鸟哥的 Linux 私房菜》系统管理相关章节
- 书籍：《深入理解计算机系统》（CSAPP）第 9-10 章
- 视频：B站搜索"Linux 进程管理"
- 博客：https://www.brendangregg.com/linuxperf.html（性能分析大全）

**检验标准**：能用命令行找出占用内存最多的进程；能解释什么是 OOM Killer。

---

### 3.3 网络基础

**学什么**：
- TCP/IP 基础：七层/四层模型、IP 地址、端口号、TCP 三次握手/四次挥手、UDP vs TCP
- 常用网络命令：`ping`、`ifconfig`/`ip addr`、`netstat`/`ss`、`curl`、`wget`、`tcpdump`
- Socket 编程基础：Python `socket` 模块实现简单的 TCP 客户端/服务器。C++ socket API 简介
- 网络调试：`telnet`、`nc`（netcat）

**为什么学**：自动驾驶系统中各模块通过网络通信（ROS2 DDS 底层就是 UDP/TCP）。调试网络问题（如传感器数据收不到、节点通信超时）是日常任务。理解 Socket 编程有助于理解 ROS2 和 Apollo 的通信层。

**学到什么程度**：
- 理解 TCP 和 UDP 的区别及各自适用场景
- 能用 Python 写一个简单的 Socket 通信程序
- 能用 `curl` 和 `tcpdump` 调试网络问题
- 理解 ROS2 DDS 底层的网络通信原理（不需要深入）

**学习时间**：3-4 天

**推荐资源**：
- 书籍：《计算机网络：自顶向下方法》（Kurose）——最经典的计算机网络教材
- 书籍：《Unix 网络编程》（Stevens）——Socket 编程圣经（可选读）
- 视频：B站搜索"计算机网络"（哈工大/湖科大的网络课评价很高）
- 实践：https://beej.us/guide/bgnet/（Beej's Network Programming Guide，免费）

**检验标准**：能用 Python Socket 实现一个简单的多客户端聊天程序。

---

### 3.4 Docker 容器化

**学什么**：Docker 基本概念（镜像/容器/仓库）。Dockerfile 编写（`FROM`、`RUN`、`COPY`、`CMD`、`ENTRYPOINT`、`WORKDIR`）。`docker build`、`docker run`、`docker exec`、`docker logs`。`docker-compose`（多容器编译）。Docker 网络和卷（volume）挂载。NVIDIA Container Toolkit（GPU 容器支持）。

**为什么学**：自动驾驶环境配置极其复杂（特定版本的 ROS2 + OpenCV + CUDA + PCL + 各种依赖），Docker 是统一开发环境的标准方案。Apollo 和 Autoware 都提供官方 Docker 镜像。不学 Docker 就无法快速搭建开发环境。

**学到什么程度**：
- 能为一个 C++ 项目编写 Dockerfile 并构建镜像
- 能用 `docker-compose` 编排多容器应用
- 能用 NVIDIA Docker 运行 GPU 容器
- 理解镜像分层机制和缓存优化

**学习时间**：3-4 天

**推荐资源**：
- 书籍：《Docker 从入门到实践》（https://yeasy.gitbook.io/docker_practice/，免费在线）
- 官方教程：https://docs.docker.com/get-started/
- 视频：TechWorld with Nana "Docker Tutorial for Beginners"（B站搬运）
- GitHub：https://github.com/ApolloAuto/apollo（Apollo 的 Dockerfile 是很好的参考）
- NVIDIA Docker：https://github.com/NVIDIA/nvidia-docker

**检验标准**：能为一个自动驾驶项目编写完整的 Dockerfile，包含 ROS2 和依赖库的安装。

---

### 3.5 tmux/screen

**学什么**：tmux 基本操作（创建会话 `tmux new -s name`、分离 `Ctrl+b d`、重新连接 `tmux attach -t name`）。窗口管理（创建窗口 `Ctrl+b c`、切换窗口 `Ctrl+b n/p`）。面板（pane）分割（水平 `Ctrl+b %`、垂直 `Ctrl+b "`）。自定义 `~/.tmux.conf`。

**为什么学**：自动驾驶开发经常需要 SSH 远程连接服务器，tmux 可以保持会话不中断（断线不丢失），可以同时查看多个终端窗口（一个跑程序、一个看日志、一个监控 GPU）。

**学到什么程度**：能熟练使用 tmux 管理多个终端会话和窗口。

**学习时间**：1 天

**推荐资源**：
- 官方文档：https://github.com/tmux/tmux/wiki
- 教程：https://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/
- 速查表：https://tmuxcheatsheet.com/
- 视频：B站搜索"tmux 使用教程"

**检验标准**：能用 tmux 创建一个包含多个窗口和面板的开发工作区。

---

### 3.6 性能分析工具

**学什么**：
- `top` / `htop`：CPU 和内存实时监控、按进程排序
- `nvidia-smi`：GPU 使用率、显存占用、GPU 温度、进程列表
- `nvidia-smi dmon`：GPU 持续监控
- `perf`：CPU 性能分析（热点函数、缓存命中率、分支预测）
- `gprof`：函数调用分析（GCC 编译时加 `-pg`）
- `valgrind --tool=callgrind`：函数调用图分析
- `ncu` / `nsight-compute`：CUDA kernel 性能分析（可选了解）

**为什么学**：自动驾驶系统有严格的实时性要求（如 100ms 内完成规划），性能分析是优化的前提。需要用这些工具定位 CPU/GPU 瓶颈。

**学到什么程度**：
- 能用 `top`/`htop` 监控系统资源
- 能用 `nvidia-smi` 监控 GPU 使用情况
- 能用 `perf` 找到程序的 CPU 热点函数
- 了解 CUDA 性能分析工具的存在和基本用途

**学习时间**：2-3 天

**推荐资源**：
- 书籍：《Systems Performance》（Brendan Gregg）
- 博客：https://www.brendangregg.com/linuxperf.html
- perf：https://perf.wiki.kernel.org/index.php/Tutorial
- NVIDIA 官方：https://developer.nvidia.com/nvidia-systems-management-interface
- 视频：Brendan Gregg "Linux Performance Tools"（YouTube）

**检验标准**：能用 `perf` 找出一个 C++ 程序中最耗时的函数；能用 `nvidia-smi` 判断 GPU 是否是瓶颈。

---

## Part 4: Git 工作流

### 4.1 基本操作

**学什么**：`git init`、`git clone`、`git add`、`git commit`、`git status`、`git log`、`git diff`。`git stash`（暂存工作区）。`git remote`（远程仓库管理）。`git fetch` vs `git pull`。`git push`。`git reset`（`--soft`、`--mixed`、`--hard`）。`git revert`。

**为什么学**：Git 是版本控制的必备工具，任何团队协作项目都离不开 Git。

**学到什么程度**：能熟练使用上述所有命令，理解 Git 的三棵树模型（工作区、暂存区、版本库）。

**学习时间**：2-3 天

**推荐资源**：
- 书籍/教程：https://git-scm.com/book/zh/v2（Pro Git 中文版，免费）
- 交互式学习：https://learngitbranching.js.org/?locale=zh_CN（极力推荐，可视化学 Git 分支）
- 视频：B站搜索"Git 教程"
- 速查表：https://education.github.com/git-cheat-sheet-education.pdf

**检验标准**：能用 `git reset` 和 `git revert` 撤销错误的提交。

---

### 4.2 分支策略与 rebase vs merge

**学什么**：分支创建、切换、合并、删除。`git merge`（产生合并提交）vs `git rebase`（线性历史）。`git cherry-pick`（捡取特定提交）。冲突解决。

**为什么学**：团队开发中每个人在自己的分支上开发，最后合并到主分支。理解 merge 和 rebase 的区别对团队协作至关重要。自动驾驶项目通常要求清晰的 commit 历史。

**学到什么程度**：
- 能熟练使用分支进行开发
- 理解 merge 和 rebase 的区别，知道各自适用场景
- 能解决合并冲突
- 知道 `rebase -i` 交互式变基（合并 commit、修改 message）

**学习时间**：2 天

**推荐资源**：
- 交互式学习：https://learngitbranching.js.org/?locale=zh_CN
- 官方文档：https://git-scm.com/book/zh/v2/Git-分支-分支的变基
- 视频：Fireship "Git Branching Strategies"

**检验标准**：能用 rebase 整理杂乱的 commit 历史；能解决复杂的合并冲突。

---

### 4.3 Git 工作流

**学什么**：
- **Gitflow**：`main`（生产）、`develop`（开发）、`feature/*`（功能）、`release/*`（发布）、`hotfix/*`（热修复）五种分支
- **Trunk-based**：所有人直接向 main 提交短生命周期分支，搭配 feature flag
- **GitHub Flow**：基于 Pull Request 的轻量级工作流
- PR/MR 的写法：标题规范、描述模板、代码审查流程

**为什么学**：自动驾驶公司通常使用其中一种工作流，了解多种工作流有助于快速适应团队。开源项目（Apollo、Autoware）都使用 PR 流程。

**学到什么程度**：理解三种工作流的优缺点，能按团队规范提交 PR。

**学习时间**：1 天

**推荐资源**：
- 文章：https://www.atlassian.com/git/tutorials/comparing-workflows
- 文章：https://trunkbaseddevelopment.com/
- GitHub Flow：https://docs.github.com/en/get-started/quickstart/github-flow

**检验标准**：能在 GitHub 上创建一个规范的 Pull Request。

---

### 4.4 .gitignore 和 .clang-format

**学什么**：
- `.gitignore`：忽略编译产物（`build/`、`*.o`、`*.so`）、IDE 文件（`.vscode/`、`.idea/`）、数据文件（`*.bag`、`*.pcd`、`*.bin`）。使用模板（https://github.com/github/gitignore）
- `.clang-format`：C++ 代码格式化规则。`clang-format` 工具使用。Google/LLVM/Mozilla 等预设风格。与 CI 集成自动格式化。

**为什么学**：代码格式统一是团队协作的基本要求。自动驾驶项目的 C++ 代码通常有严格的格式规范（如 Apollo 使用 Google 风格）。

**学到什么程度**：
- 能为项目配置完整的 `.gitignore`
- 能配置 `.clang-format` 并用 `clang-format` 工具格式化代码
- 能在 VSCode 中配置保存时自动格式化

**学习时间**：1 天

**推荐资源**：
- .gitignore 模板：https://github.com/github/gitignore
- clang-format 配置生成器：https://zed0.co.uk/clang-format-configurator/
- VSCode 插件：C/C++ (Microsoft) 扩展自带 clang-format 支持
- Apollo 的 .clang-format：https://github.com/ApolloAuto/apollo/blob/master/.clang-format

**检验标准**：能为一个新项目配置 .gitignore 和 .clang-format，并在 VSCode 中启用自动格式化。

---

### 4.5 CI/CD 基础概念

**学什么**：CI（持续集成）的概念（每次提交自动构建和测试）。CD（持续部署/交付）。GitHub Actions 基本语法（`.github/workflows/` YAML 文件）。常见 CI 步骤（checkout、build、test、lint）。Docker 镜像自动构建。

**为什么学**：自动驾驶公司都有 CI/CD 流水线，提交代码后自动触发构建、单元测试、集成测试。理解 CI/CD 有助于理解团队开发流程。

**学到什么程度**：能为一个 C++ 项目配置基本的 GitHub Actions CI（自动构建 + 运行测试）。

**学习时间**：2-3 天

**推荐资源**：
- GitHub Actions 官方文档：https://docs.github.com/en/actions
- 教程：https://docs.github.com/en/actions/quickstart
- 视频：Fireship "GitHub Actions in 100 Seconds"
- 示例：搜索自动驾驶开源项目的 GitHub Actions 配置

**检验标准**：能为一个 CMake 项目配置 GitHub Actions，每次 push 自动编译并运行单元测试。

---

## Part 5: 数学基础

> 注意：你已有自动控制原理和现代控制理论基础，很多数学工具（状态空间、矩阵运算、拉普拉斯变换）已经学过。以下内容侧重自动驾驶场景特有的数学需求，会在适当位置与你的控制理论基础建立联系。

### 5.1 线性代数（进阶）

#### 5.1.1 矩阵分解（SVD、QR、Cholesky）

**学什么**：
- **SVD（奇异值分解）**：$A = U\Sigma V^T$，理解几何意义（旋转-缩放-旋转）。SVD 在最小二乘、伪逆、PCA、数据降维中的应用。
- **QR 分解**：$A = QR$，正交矩阵 × 上三角矩阵。QR 分解在求解线性方程组和最小二乘中的应用。
- **Cholesky 分解**：$A = LL^T$（正定矩阵），高效求解正定线性方程组。在卡尔曼滤波中的应用（协方差矩阵传播）。
- **LU 分解**：$A = LU$，求解一般线性方程组。

**为什么学**：
- SVD 是自动驾驶中最重要的矩阵分解——用于点云配准（ICP 算法内部用 SVD 求最优旋转）、数据降维、伪逆计算。
- Cholesky 分解在卡尔曼滤波的协方差传播中效率更高（你已有控制理论基础，很快能理解）。
- QR 分解在数值稳定的最小二乘求解中使用。

**学到什么程度**：
- 能手算 2x2 或 3x3 矩阵的 SVD
- 理解 SVD 的几何意义，知道如何用 SVD 求解最小二乘问题
- 理解 Cholesky 分解在卡尔曼滤波中的作用
- 能用 NumPy/Eigen 实现这些分解

**学习时间**：4-5 天

**推荐资源**：
- 书籍：《线性代数应该这样学》（Sheldon Axler）——偏理论但讲得好
- 书籍：《矩阵分析与应用》（张贤达）——国内经典
- 视频：3Blue1Brown "Essence of Linear Algebra"（B站有字幕版，极力推荐）
- 视频：Gilbert Strang MIT 18.06 线性代数（B站搬运）
- 博客：https://www.cnblogs.com/pinard/p/6251450.html（刘建平的 SVD 讲解）
- NumPy 文档：https://numpy.org/doc/stable/reference/generated/numpy.linalg.svd.html

**检验标准**：能用 SVD 求解一个点云配准问题（给定两组对应点，求最优旋转和平移）。

---

#### 5.1.2 特征值与特征向量

**学什么**：特征值/特征向量的定义和计算。对称矩阵的特征分解（谱分解）。特征值的几何意义。PCA（主成分分析）的数学原理。矩阵幂的特征值解释。

**为什么学**：自动驾驶中 PCA 用于点云降维和方向估计（OBB 包围盒计算）。特征值分析在稳定性分析（你已学过的控制理论中的极点分析就是特征值应用）中至关重要。

**学到什么程度**：能手算 2x2/3x3 矩阵的特征值和特征向量。能用 PCA 做点云主方向分析。

**学习时间**：2 天

**推荐资源**：
- 视频：3Blue1Brown "Eigenvectors and Eigenvalues"
- 书籍：同上 5.1.1

**检验标准**：能用 NumPy 的 PCA 分析一组点云数据的主方向。

---

#### 5.1.3 齐次坐标与变换矩阵

**学什么**：齐次坐标的定义（$(x,y,z)$ 变为 $(x,y,z,1)$）。为什么要用齐次坐标（统一旋转和平移为矩阵乘法）。SE(3) 变换矩阵的结构（4x4 矩阵）。逆变换。变换的复合（矩阵连乘）。2D 齐次坐标（3x3）。

**为什么学**：自动驾驶中所有的坐标变换（传感器坐标系 -> 车辆坐标系 -> 世界坐标系）都使用齐次坐标和变换矩阵。不理解齐次坐标就无法理解自动驾驶中的坐标系变换。

**学到什么程度**：
- 能写出任意旋转+平移的 4x4 变换矩阵
- 能计算变换矩阵的逆
- 能计算多个变换的复合
- 能用代码实现坐标系间的点云变换

**学习时间**：2 天

**推荐资源**：
- 书籍：《视觉SLAM十四讲》（高翔）第 3 章——极力推荐，从工程角度讲得很清楚
- 书籍：《State Estimation for Robotics》（Tim Barfoot）第 1-2 章
- 视频：3Blue1Brown "Linear Transformations and Matrices"
- 博客：https://www.cnblogs.com/gaoxiang12/p/5136710.html

**检验标准**：能将激光雷达点云从雷达坐标系变换到车辆坐标系，再变换到世界坐标系。

---

#### 5.1.4 旋转矩阵与四元数

**学什么**：
- **旋转矩阵**：SO(3) 的性质（正交、行列式为 1）。旋转矩阵的 9 个参数只有 3 个自由度。
- **欧拉角**：roll/pitch/yaw，万向锁（Gimbal Lock）问题。
- **轴角表示**：绕某一轴旋转一定角度。
- **四元数**：$q = w + xi + yj + zk$，单位四元数表示旋转。四元数的运算（乘法、共轭、归一化）。四元数与旋转矩阵的相互转换。SLERP（球面线性插值）。
- 各种旋转表示的优缺点和互相转换。

**为什么学**：自动驾驶中 IMU 输出的是欧拉角或四元数，车辆朝向用四元数存储（避免万向锁），SLAM 中优化用李代数。理解各种旋转表示及其转换是必须的。

**学到什么程度**：
- 能在旋转矩阵、欧拉角、四元数之间手动转换
- 理解万向锁的原理
- 能用四元数做旋转插值（SLERP）
- 能用 Eigen 或 NumPy 实现各种转换

**学习时间**：4-5 天（这是重点难点）

**推荐资源**：
- 书籍：《视觉SLAM十四讲》第 3 章——四元数和旋转讲得非常好
- 书籍：《State Estimation for Robotics》（Tim Barfoot）
- 视频：3Blue1Brown "Quaternions and 3d rotation"（四元数可视化讲得最好的）
- 网站：https://eater.net/quaternions（交互式四元数学习网站）
- 博客：https://www.zhihu.com/question/23005815（知乎四元数精华帖）
- 工具：https://quaternion.shivamkundra.dev/（四元数可视化工具）

**检验标准**：能用四元数实现一个点绕任意轴旋转的计算；能在 IMU 数据处理中正确转换四元数和旋转矩阵。

---

### 5.2 概率统计

#### 5.2.1 贝叶斯公式

**学什么**：条件概率。贝叶斯公式 $P(A|B) = \frac{P(B|A)P(A)}{P(B)}$。先验/后验/似然/证据的含义。贝叶斯推理的基本框架。

**为什么学**：贝叶斯推理是自动驾驶状态估计的数学基础——卡尔曼滤波、粒子滤波、SLAM 后端优化本质上都是贝叶斯推理。你学过的卡尔曼滤波就是贝叶斯滤波在线性高斯假设下的特例。

**学到什么程度**：能用贝叶斯公式解决基本的推断问题，理解先验如何通过观测更新为后验。

**学习时间**：1 天（你有控制理论基础，应该很快）

**推荐资源**：
- 书籍：《概率机器人》（Probabilistic Robotics，Thrun）第 2 章——自动驾驶概率方法的经典教材
- 书籍：《Bayesian Reasoning and Machine Learning》（David Barber）——免费 PDF
- 视频：3Blue1Brown "Bayes theorem"（B站搬运）
- 博客：https://www.yanxurui.cc/posts/statistics/2017-02-05-bayes-theorem/

**检验标准**：能用贝叶斯公式分析一个自动驾驶场景（如传感器融合中的置信度更新）。

---

#### 5.2.2 高斯分布（正态分布）

**学什么**：一元/多元高斯分布的概率密度函数。均值向量和协方差矩阵的含义。高斯分布的性质（线性变换仍是高斯、边缘分布仍是高斯、条件分布仍是高斯）。高斯分布的 KL 散度。

**为什么学**：卡尔曼滤波假设噪声是高斯的，状态分布是高斯的。自动驾驶中位置估计的不确定性通常用高斯分布（均值+协方差）表示。理解高斯分布是理解卡尔曼滤波的前提。

**学到什么程度**：
- 能写出多元高斯分布的公式
- 理解协方差矩阵的几何含义（椭圆形状表示不确定性方向和大小）
- 能用 NumPy 采样和计算高斯分布
- 理解高斯分布在非线性变换后不再是高斯（EKF 就是为了近似处理这个问题）

**学习时间**：2 天

**推荐资源**：
- 书籍：《概率机器人》（Thrun）第 2 章
- 视频：3Blue1Brown "But what is a Gaussian distribution?"（YouTube）
- 博客：https://cs229.stanford.edu/section/gaussians.pdf（Stanford CS229 高斯分布笔记）

**检验标准**：能画出 2D 高斯分布的等高线图，能解释协方差矩阵的特征值与椭圆长短轴的关系。

---

#### 5.2.3 MLE 和 MAP

**学什么**：最大似然估计（MLE）：$\hat{\theta}_{MLE} = \arg\max_\theta P(D|\theta)$。最大后验估计（MAP）：$\hat{\theta}_{MAP} = \arg\max_\theta P(\theta|D)$。MAP 与正则化的联系（L2 正则化对应高斯先验）。

**为什么学**：自动驾驶中的许多优化问题都可以从 MLE/MAP 的角度理解——传感器标定（MLE）、SLAM 中的位姿优化（MAP，带有先验约束）、点云配准（MLE）。

**学到什么程度**：
- 能推导高斯分布下的 MLE 解（就是样本均值和样本协方差）
- 理解 MAP 中先验的作用
- 能用 MLE/MAP 的框架理解优化问题

**学习时间**：2 天

**推荐资源**：
- 书籍：《概率机器人》（Thrun）
- 课程：Stanford CS229 机器学习课程（B站有搬运）
- 视频：3Blue1Brown + StatQuest 的统计学系列
- 博客：https://wiseodd.github.io/techblog/2017/01/01/mle-vs-map/

**检验标准**：能从 MLE 的角度推导线性回归的解析解。

---

#### 5.2.4 马尔可夫链

**学什么**：马尔可夫性质（未来只取决于当前状态）。马尔可夫链的状态转移矩阵。平稳分布。隐马尔可夫模型（HMM）基础。马尔可夫链蒙特卡洛（MCMC）基本思想。

**为什么学**：自动驾驶中的运动预测（预测其他车辆的未来行为）建立在马尔可夫假设上。卡尔曼滤波也假设系统是马尔可夫的（当前状态只取决于上一时刻）。MCMC 用于处理复杂后验分布的采样。

**学到什么程度**：
- 理解马尔可夫性质和转移矩阵
- 能计算简单的平稳分布
- 了解 HMM 的基本结构（状态、观测、转移概率、发射概率）
- 了解 MCMC 的基本思想（Metropolis-Hastings 算法的概念）

**学习时间**：2 天

**推荐资源**：
- 书籍：《概率机器人》
- 视频：3Blue1Brown "Markov Chains"（YouTube）
- 课程：Stanford CS229 或 CS228 概率图模型课程
- 博客：https://www.cnblogs.com/pinard/p/6632399.html（马尔可夫链讲解）

**检验标准**：能用马尔可夫链模型描述一个简单的交通灯状态切换过程，并计算平稳分布。

---

### 5.3 最优化

#### 5.3.1 梯度下降及其变体

**学什么**：
- **梯度下降（GD）**：$\theta \leftarrow \theta - \alpha \nabla J(\theta)$，学习率选择，收敛条件。
- **随机梯度下降（SGD）**：每次用一个样本估计梯度，噪声大但能跳出局部最优。
- **Mini-batch SGD**：折中方案，batch size 的影响。
- **SGD 变体**：Momentum（带动量）、RMSProp（自适应学习率）、Adam（动量+自适应）。Adam 的公式和超参数（$\beta_1$, $\beta_2$, $\epsilon$）。
- 学习率调度（Learning Rate Schedule）：Step Decay、Cosine Annealing、Warmup。

**为什么学**：深度学习训练（感知模型、预测模型）全部使用梯度下降的变体。自动驾驶的感知模型训练需要理解这些优化器的特性来调参。控制优化中的梯度优化也需要这些基础。

**学到什么程度**：
- 能手推梯度下降的更新公式
- 能用 NumPy 实现一个简单的 SGD
- 理解 Adam 的原理和优势
- 知道如何选择学习率和 batch size
- 不需要推导所有变体的收敛性证明

**学习时间**：3-4 天

**推荐资源**：
- 书籍：《深度学习》（Goodfellow）第 8 章——优化章节写得非常好
- 视频：Stanford CS231n 优化部分（B站搬运）
- 博客：https://ruder.io/optimizing-gradient-descent/（Sebastian Ruder 的经典总结）
- 可视化：http://cs231n.stanford.edu/（CS231n 课程笔记）
- 工具：https://losslandscape.com/（损失面可视化）

**检验标准**：能用 NumPy 从零实现带 Momentum 的 SGD，训练一个简单的神经网络。

---

#### 5.3.2 拉格朗日乘子法

**学什么**：等式约束优化问题的拉格朗日乘子法。拉格朗日函数 $L(x, \lambda) = f(x) + \lambda g(x)$。对 $x$ 和 $\lambda$ 求导设为零。不等式约束和 KKT 条件。

**为什么学**：自动驾驶中的很多优化问题有约束——控制量有上下界（油门 0~1）、车辆运动学约束、避障约束等。拉格朗日乘子法是处理约束优化的基本工具。你学过的最优控制理论中的 LQR 也可以从拉格朗日角度理解。

**学到什么程度**：
- 能用拉格朗日乘子法求解等式约束优化问题
- 理解 KKT 条件的四个条件
- 知道 KKT 条件是约束优化最优解的必要条件
- 能用拉格朗日乘子法推导支持向量机（SVM）

**学习时间**：2-3 天

**推荐资源**：
- 书籍：《最优化导论》（Edwin Chong）——适合工科生
- 视频：Stanford CS229 第六讲（B站搬运）
- 博客：https://www.cnblogs.com/pinard/p/5976811.html
- 书籍：《Convex Optimization》（Boyd）第 4-5 章（在线免费：https://web.stanford.edu/~boyd/cvxbook/）

**检验标准**：能用 KKT 条件求解一个简单的约束优化问题。

---

#### 5.3.3 凸优化基础

**学什么**：凸集、凸函数、凸优化问题的定义。凸优化的性质（局部最优即全局最优）。常见的凸优化问题类型（LP、QP、SOCP、SDP）。对偶问题。强对偶性。

**为什么学**：自动驾驶中的 MPC（模型预测控制）可以建模为凸优化问题（QP），用求解器高效求解。路径平滑化、速度规划等问题也可以建模为凸优化。凸优化理论保证找到全局最优，这对安全性至关重要。

**学到什么程度**：
- 能判断一个优化问题是否是凸的
- 理解凸优化问题的标准形式
- 能将 MPC 问题转化为 QP 形式
- 了解常用的凸优化求解器（OSQP、Gurobi、CVXPY）

**学习时间**：3-4 天

**推荐资源**：
- 书籍：《Convex Optimization》（Boyd & Vandenberghe）——经典教材，免费 PDF
- 课程：Stanford EE364a 凸优化课程（B站有搬运）
- Python 工具：CVXPY（https://www.cvxpy.org/）——用 Python 建模凸优化问题
- 博客：https://www.cvxgrp.org/（Stanford Boyd 组的研究博客）

**检验标准**：能用 CVXPY 建模并求解一个简单的 MPC 问题。

---

### 5.4 李群与李代数

**学什么**：
- **SO(3)**：三维旋转群，特殊正交群。李群的性质（群运算封闭、结合律、单位元、逆元）。
- **SE(3)**：三维刚体运动群（旋转+平移）。
- **李代数 so(3) 和 se(3)**：李群的切空间。so(3) 与三维向量的对应关系（$\omega$ 叉乘矩阵）。
- **指数映射**：$\exp: \mathfrak{so}(3) \to SO(3)$，Rodrigues 公式。从角速度到旋转矩阵的映射。
- **对数映射**：$\log: SO(3) \to \mathfrak{so}(3)$，从旋转矩阵到旋转向量的映射。
- **BCH 公式近似**：理解为什么在李代数空间做加法更方便（优化时用小量更新）。

**为什么学**：SLAM（同时定位与建图）的核心算法——图优化（g2o、GTSAM）在李代数空间进行优化。自动驾驶的定位模块（LIO、VIO）都依赖李群李代数。在 SO(3) 上无法直接做加法（旋转矩阵相加不再是旋转矩阵），必须在李代数空间操作。

**学到什么程度**：
- 能手算 SO(3) 的指数映射（Rodrigues 公式）
- 能手算对数映射（从旋转矩阵提取旋转向量）
- 理解为什么优化变量选择李代数而不是旋转矩阵或欧拉角
- 能用 Eigen 的 `AngleAxis` 和 Sophus 库进行李群运算
- 不需要严格的数学证明，但要理解直觉

**学习时间**：5-7 天（这是难点，需要反复理解）

**推荐资源**：
- 书籍：《视觉SLAM十四讲》（高翔）第 4 章——极力推荐，对工程背景最友好
- 书籍：《State Estimation for Robotics》（Tim Barfoot）第 7 章——更数学但更完整，免费 PDF
- 书籍：《Lie Groups for 2D and 3D Transformations》（Ethan Eade）——简洁的李群参考手册，免费 PDF
- GitHub：https://github.com/strasdat/Sophus（Sophus 李群库，有详细注释）
- 视频：高翔 SLAM 十四讲配套视频（B站搜索"SLAM十四讲"）
- 博客：https://www.cnblogs.com/gaoxiang12/p/5140837.html

**检验标准**：能用 Rodrigues 公式将一个旋转向量转换为旋转矩阵，反过来也行。能解释为什么 SLAM 优化选择在李代数空间操作。

---

### 5.5 卡尔曼滤波（EKF、UKF）

> 你已有自动控制原理和现代控制理论基础，这对你理解卡尔曼滤波非常有帮助。卡尔曼滤波本质上就是线性系统状态估计，与你学过的状态观测器（Luenberger Observer）有密切联系。

#### 5.5.1 线性卡尔曼滤波（复习与深化）

**学什么**：
- 状态空间模型（离散时间）：$x_k = F x_{k-1} + B u_k + w_k$，$z_k = H x_k + v_k$
- 预测步（Predict）和更新步（Update）的五个公式
- 协方差矩阵 $P$ 的含义（不确定性）
- 卡尔曼增益 $K$ 的直觉理解（测量可信则 K 大，预测可信则 K 小）
- 与你学过的状态观测器（Luenberger Observer）的联系——卡尔曼增益对应观测器增益
- 发散问题和协方差矩阵的正定性

**为什么学**：这是理解 EKF/UKF 和更高级滤波器的基础。自动驾驶中的 GPS/IMU 融合、轮速里程计融合等场景直接使用线性卡尔曼滤波。

**学到什么程度**：
- 能手推卡尔曼滤波的五个公式
- 能用代码实现简单的卡尔曼滤波器
- 能解释卡尔曼增益的物理意义
- 与控制理论中的状态观测器建立联系

**学习时间**：2-3 天（有控制基础应该较快）

**推荐资源**：
- 书籍：《概率机器人》（Thrun）第 3 章——自动驾驶领域最权威
- 书籍：《最优状态估计》（Simon）——偏工程
- 书籍：《State Estimation for Robotics》（Barfoot）第 3 章——免费 PDF
- 视频：Steve Brunton "Kalman Filter"（YouTube/B站搬运）——直观理解讲得最好
- 视频：3Blue1Brown 贝叶斯推断视频
- 网站：https://www.bzarg.com/p/a-technical-introduction-to-the-kalman-filter/（经典入门文章）
- Python 实现：https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python（免费在线书+Jupyter Notebook，极力推荐）

**检验标准**：能用卡尔曼滤波器融合 GPS 和 IMU 数据估计车辆位置。

---

#### 5.5.2 扩展卡尔曼滤波（EKF）

**学什么**：
- 非线性系统模型：$x_k = f(x_{k-1}, u_k) + w_k$，$z_k = h(x_k) + v_k$
- 雅可比矩阵（Jacobian）：$F = \frac{\partial f}{\partial x}\Big|_{\hat{x}}$，$H = \frac{\partial h}{\partial x}\Big|_{\hat{x}}$
- EKF 的预测步和更新步（用雅可比矩阵替代线性卡尔曼的 F 和 H）
- EKF 的局限性：线性化误差、雅可比矩阵计算复杂、可能发散
- 一阶泰勒展开的近似

**为什么学**：自动驾驶中大部分系统是非线性的——车辆运动学是非线性的，传感器观测模型（如雷达的距离-角度观测）也是非线性的。EKF 是处理非线性状态估计的经典方法。LiDAR-INS 紧耦合定位中的误差状态卡尔曼滤波器（ESKF）就是基于 EKF。

**学到什么程度**：
- 能手动推导 EKF 的雅可比矩阵（至少能推导简单的非线性系统）
- 能用代码实现 EKF
- 理解线性化的含义和误差来源
- 知道 EKF 的适用条件和局限性

**学习时间**：3-4 天

**推荐资源**：
- 书籍：《概率机器人》（Thrun）第 3-4 章
- 书籍：《State Estimation for Robotics》（Barfoot）第 3-4 章
- 书籍：《最优状态估计》（Simon）第 5 章
- GitHub：https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python 第 8-10 章（EKF 详解+代码）
- 视频：Steve Brunton "Extended Kalman Filter"
- 博客：https://www.bzarg.com/p/how-a-kalman-filter-works-in-pics/

**检验标准**：能用 EKF 实现一个简单的车辆状态估计（非线性运动模型 + 雷达观测）。

---

#### 5.5.3 无迹卡尔曼滤波（UKF）

**学什么**：
- UKF 的核心思想：用 sigma 点来表示分布，而不是线性化非线性函数
- Sigma 点的选取方法（2n+1 个点，n 是状态维度）
- 无迹变换（Unscented Transform）：sigma 点通过非线性函数传播后，重新估计均值和协方差
- UKF 的预测步和更新步
- UKF vs EKF：不需要计算雅可比矩阵，精度更高（二阶近似 vs 一阶近似）

**为什么学**：UKF 在很多自动驾驶场景中比 EKF 效果更好——对于高度非线性的系统，EKF 的线性化误差可能很大。自动驾驶中的多传感器融合系统（如 LiDAR-Camera-IMU 融合）有时会使用 UKF。Sigma 点的思想也用于机器人定位中的 Monte Carlo 方法。

**学到什么程度**：
- 理解 sigma 点的选取方式和权重计算
- 能用代码实现 UKF
- 能比较 UKF 和 EKF 在同一非线性系统上的精度差异
- 理解 UKF 不需要雅可比矩阵的优势

**学习时间**：2-3 天

**推荐资源**：
- 书籍：《概率机器人》
- 书籍：《State Estimation for Robotics》（Barfoot）
- GitHub：https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python 第 11 章
- 论文：Julier & Uhlmann "Unscented Filtering and Nonlinear Estimation"（经典论文）
- 博客：https://towardsdatascience.com/the-unscented-kalman-filter-anything-ekf-can-do-i-can-do-it-better-ce7c357d6d35

**检验标准**：能用 UKF 处理一个比 EKF 更高度非线性的问题（如带角度观测的定位），并展示 UKF 相比 EKF 的精度优势。

---

### 补充说明：学习路径建议

**第一阶段（1-2 个月）—— 打基础**：
- C++ 核心特性（智能指针、移动语义、Lambda、多线程）
- CMake
- NumPy + Pandas
- 线性代数进阶（齐次坐标、旋转、SVD）
- Linux 基本操作 + Git

**第二阶段（2-3 个月）—— 深入**：
- C++ 设计模式 + 模板基础
- 线程池 + 多线程进阶
- 李群李代数
- 概率统计 + 卡尔曼滤波
- Docker + 网络基础
- LeetCode 持续刷题

**第三阶段（1-2 个月）—— 巩固**：
- 最优化理论
- Python 高级特性
- 性能分析工具
- 完整项目实践（如用 C++ 实现一个简单的 EKF 定位模块，用 CMake 构建，Docker 打包，Git 管理）

**核心原则**：
1. 每个知识点学完后立即动手写代码实践，不能只看书
2. 结合自动驾驶场景选择练习项目
3. LeetCode 持续刷，不要停
4. 多读开源代码（Apollo、Autoware.universe），对照学习指南中的知识点在代码中的实际应用

---

# 模块二：深度学习与计算机视觉（2D感知）

# 自动驾驶方向 - 深度学习与计算机视觉（2D感知）完整学习指南

> 适用对象：机器人工程本科背景、考研专业课为自动控制原理/现代控制理论、深度学习零基础的准研究生
> 目标定位：工程落地型，以"能跑通代码、能训练模型、能上手实际项目"为导向

---

## Part 1: 深度学习基础（从零到能用）

### 1.1 神经网络基本原理（前向传播、反向传播推导）

**学什么**：

- 生物神经元到人工神经元的类比：输入信号、权重、偏置、激活函数、输出
- 单个感知机（Perceptron）的数学表达：$y = f(\mathbf{w}^T\mathbf{x} + b)$
- 多层感知机（MLP）的结构：输入层、隐藏层、输出层，每层之间的全连接关系
- 前向传播（Forward Propagation）：数据从输入层逐层计算到输出层的完整过程，包括每层的线性变换 $z^{[l]} = W^{[l]}a^{[l-1]} + b^{[l]}$ 和非线性激活 $a^{[l]} = g(z^{[l]})$
- 损失函数（Loss Function）的定义：衡量模型预测值与真实值之间的差距
- 反向传播（Backpropagation）的核心推导：
  - 链式法则（Chain Rule）：这是你学控制理论时接触过的复合函数求导
  - 从输出层到输入层，逐层计算损失对每个参数的偏导数 $\frac{\partial L}{\partial W^{[l]}}$ 和 $\frac{\partial L}{\partial b^{[l]}}$
  - 计算图（Computational Graph）的概念：将前向和反向传播可视化为有向无环图
- 梯度下降（Gradient Descent）：$\theta \leftarrow \theta - \eta \cdot \nabla_\theta L$
  - 批量梯度下降（Batch GD）、随机梯度下降（SGD）、小批量梯度下降（Mini-batch GD）的区别

**为什么学**：

这是整个深度学习的地基。后续所有的CNN、Transformer、检测模型都建立在"前向计算 -> 计算损失 -> 反向传播 -> 更新参数"这个循环之上。你学过的控制理论中"系统建模 -> 误差计算 -> 控制律 -> 系统更新"的思路与此高度类似，可以帮助你建立直觉。

**学到什么程度**：

- 能手推一个两层MLP的前向传播和反向传播全过程（包括矩阵维度分析）
- 能用纯NumPy手写一个两层神经网络在MNIST数据集上完成分类（不用任何深度学习框架）
- 理解梯度消失和梯度爆炸问题及其与激活函数的关系
- 理解为什么需要非线性激活函数（如果没有会退化为线性模型）

**学习时间**：1-2周

**推荐资源**：

- 书籍：
  - 《深度学习》（花书）Ian Goodfellow 著，第6章"深度前馈网络"、第8章"深度学习优化"
  - 《动手学深度学习》（d2l.ai）李沐 著，第4章"多层感知机"——强烈推荐，中文免费在线版
- 课程：
  - 吴恩达《Deep Learning Specialization》第1门课"Neural Networks and Deep Learning"（Coursera，B站有搬运：https://www.bilibili.com/video/BV1FT4y1E74V）
  - 李宏毅《机器学习》2021版 前几讲（B站：https://www.bilibili.com/video/BV1Wv411h7kN）
  - 李沐《动手学深度学习》配套视频（B站：https://space.bilibili.com/1567748478）
- GitHub：
  - `microsoft/ai-edu` —— 微软人工智能教育，有中文教程和代码
  - `dsgiitr/d2l-pytorch` —— 动手学深度学习的PyTorch实现笔记
- 博客/教程：
  - 3Blue1Brown 神经网络系列（B站有中文字幕版，搜索"3Blue1Brown 神经网络"）—— 可视化讲解反向传播，极其推荐
  - CS231n 课程笔记"Backpropagation"部分（英文，https://cs231n.github.io/optimization-2/）

**检验标准**：

- 能在纸上手推一个2-3层网络的前向和反向传播，维度完全正确
- 能用NumPy从零实现一个两层网络，MNIST准确率 > 95%
- 能清晰解释"为什么反向传播比数值梯度高效"

---

### 1.2 CNN架构演进

#### 1.2.1 卷积操作与基本概念

**学什么**：

- 二维卷积的数学定义：输入特征图与卷积核的滑动点积操作
- 关键超参数：卷积核大小（kernel size）、步长（stride）、填充（padding）、膨胀率（dilation）
- 输出尺寸公式：$H_{out} = \lfloor\frac{H_{in} + 2 \cdot padding - dilation \cdot (kernel - 1) - 1}{stride} + 1\rfloor$
- 通道数变化：输入通道数 $C_{in}$、输出通道数 $C_{out}$、参数量 = $C_{in} \times C_{out} \times k \times k$
- 感受野（Receptive Field）的概念及其递增规律
- 池化层（Pooling）：最大池化、平均池化、全局平均池化
- 1x1卷积的作用：通道数变换（降维/升维）
- 深度可分离卷积（Depthwise Separable Convolution）：Depthwise + Pointwise，参数量大幅减少

**为什么学**：

卷积操作是CNN的核心算子，后续所有图像分类、目标检测、分割模型都以卷积为基础。自动驾驶中摄像头图像的处理几乎全部基于卷积神经网络。

**学到什么程度**：

- 能手算一个简单卷积操作的输出（给定输入、kernel、stride、padding）
- 能用PyTorch的 `nn.Conv2d` 实现任意卷积层，理解每个参数的含义
- 理解参数量和计算量（FLOPs）的计算方法

**学习时间**：2-3天

**推荐资源**：

- 书籍：《动手学深度学习》第7章"卷积神经网络"
- 课程：CS231n Lecture 5 "Convolutional Neural Networks"（B站搜索"CS231n 中文字幕"）
- 博客：卷积可视化网站 https://github.com/vdumoulin/conv_arithmetic （动态图展示各种卷积操作）

#### 1.2.2 LeNet-5（1998）

**学什么**：

- 结构：2个卷积层 + 3个全连接层，用于手写数字识别（MNIST）
- 核心贡献：验证了"卷积 -> 池化 -> 全连接"这一基本范式的有效性
- 感受野逐步增大的设计思想

**为什么学**：

作为CNN的开山之作，理解它能帮你建立CNN的基本结构直觉。但不需要花太多时间，主要是了解历史脉络。

**学到什么程度**：

- 能画出LeNet-5的结构图，知道每层的输入输出尺寸
- 能用PyTorch复现

**学习时间**：半天

**推荐资源**：

- 原论文：LeCun et al., "Gradient-based learning applied to document recognition" (1998)
- 《动手学深度学习》7.6节 "LeNet"
- GitHub：`pytorch/examples/mnist` —— PyTorch官方MNIST示例

#### 1.2.3 AlexNet（2012）

**学什么**：

- 核心创新点：
  - 首次在大规模数据集（ImageNet）上用GPU训练CNN
  - 使用ReLU激活函数替代Sigmoid（解决了梯度消失问题，训练速度提升6倍）
  - 使用Dropout防止过拟合（0.5的Dropout率）
  - 数据增强：随机裁剪、水平翻转、颜色抖动
  - 局部响应归一化（LRN，后来被证明效果不大，被BatchNorm取代）
- 网络结构：5个卷积层 + 3个全连接层，约6000万参数

**为什么学**：

AlexNet是深度学习革命的起点，它证明了深度CNN在视觉任务上的巨大潜力。理解它的每个设计选择，有助于理解后续架构改进的动机。

**学到什么程度**：

- 能画出AlexNet的结构，知道ReLU、Dropout在其中的作用
- 理解为什么它比传统方法（SVM + HOG特征）好那么多

**学习时间**：半天

**推荐资源**：

- 原论文：Krizhevsky et al., "ImageNet Classification with Deep Convolutional Neural Networks" (2012)
- 《动手学深度学习》7.4节

#### 1.2.4 VGGNet（2014）

**学什么**：

- 核心创新点：
  - 全部使用3x3小卷积核（替代之前的大卷积核如7x7、11x11）
  - 堆叠多个3x3卷积等于一个大卷积核的感受野（两个3x3 = 一个5x5），但参数更少、非线性更强
  - 网络结构非常规整：每阶段用2-3个卷积 + 1个最大池化
  - VGG-16和VGG-19两个经典配置
- 参数量巨大（1.38亿），主要在全连接层

**为什么学**：

VGG确立了"用小卷积核堆叠深度"的设计原则，这个原则一直沿用至今。VGG的特征提取能力非常强，至今仍被用作某些任务的backbone（如风格迁移中的VGG特征）。

**学到什么程度**：

- 理解"两个3x3卷积替代一个5x5"的数学推导（感受野等价，参数从 $25C^2$ 降到 $18C^2$）
- 能用PyTorch实现VGG-16
- 知道VGG的优缺点（太深参数多，但特征表达能力强）

**学习时间**：1天

**推荐资源**：

- 原论文：Simonyan & Zisserman, "Very Deep Convolutional Networks for Large-Scale Image Recognition" (2014)
- 《动手学深度学习》7.4节
- GitHub：`pytorch/vision` 中的 `torchvision.models.vgg`

#### 1.2.5 GoogLeNet / Inception（2014）

**学什么**：

- 核心创新点：
  - Inception模块：在同一层中并行使用1x1、3x3、5x5卷积和3x3池化，然后在通道维度拼接（concat）
  - 1x1卷积用于降维：在3x3和5x5卷积之前加1x1卷积减少通道数，大幅降低计算量
  - 去掉了全连接层，用全局平均池化（GAP）替代，参数量从AlexNet的6000万降到约500万
  - 多尺度特征提取的思路：不同大小的卷积核捕获不同尺度的信息
- 后续改进：Inception v2/v3（引入BatchNorm、卷积核分解）、Inception v4（结合残差连接）

**为什么学**：

Inception模块的"多分支 + 1x1降维"思想影响深远。1x1卷积降维技巧在后续很多网络中被广泛使用。

**学到什么程度**：

- 能画出Inception模块的结构图，理解1x1卷积在其中的降维作用
- 理解为什么全局平均池化可以替代全连接层
- 不需要背诵每层通道数

**学习时间**：1天

**推荐资源**：

- 原论文：Szegedy et al., "Going Deeper with Convolutions" (2014)
- 《动手学深度学习》7.4节
- 李沐论文精读视频（B站搜索"李沐 GoogLeNet"）

#### 1.2.6 ResNet（2015）—— 里程碑式的工作

**学什么**：

- 核心问题：网络深度增加时，训练误差反而上升（退化问题，degradation problem），这不是过拟合，是训练本身变困难了
- 核心创新 —— 残差连接（Residual Connection）：
  - 不直接学习映射 $H(x)$，而是学习残差 $F(x) = H(x) - x$，输出为 $F(x) + x$
  - 梯度可以通过跳跃连接直接回传，解决了深层网络的梯度消失问题
  - 数学直觉：即使 $F(x)$ 的梯度很小，$x$ 的梯度始终为1，梯度不会消失
- 两种残差块：
  - Basic Block：两个3x3卷积 + 跳跃连接（ResNet-18/34）
  - Bottleneck Block：1x1降维 -> 3x3卷积 -> 1x1升维 + 跳跃连接（ResNet-50/101/152）
- 为什么是里程碑：
  - 使得训练数百甚至上千层的网络成为可能
  - 残差连接的思想被几乎所有后续架构采用（包括Transformer）
  - ResNet至今仍是目标检测、分割等任务中最常用的backbone之一
  - 2016年ILSVRC冠军，top-5错误率3.57%

**为什么学**：

ResNet是深度学习历史上最重要的架构创新之一。残差连接是后续所有主流架构（包括Vision Transformer）的标配。在自动驾驶感知中，ResNet-50是最常用的backbone之一。

**学到什么程度**：

- 能手写ResNet的Bottleneck Block和Basic Block的PyTorch代码
- 能清晰解释"为什么残差连接能解决退化问题"（从梯度流动角度）
- 理解Bottleneck中1x1卷积的作用（降维减计算量，然后升维恢复通道数）
- 能用PyTorch复现ResNet-18/50

**学习时间**：2-3天

**推荐资源**：

- 原论文：He et al., "Deep Residual Learning for Image Recognition" (2015)—— 必读论文
- 李沐论文精读："ResNet论文精读"（B站：https://www.bilibili.com/video/BV1P34y1S7FC）—— 强烈推荐，逐段讲解
- 《动手学深度学习》7.6节 "残差网络（ResNet）"
- GitHub：`pytorch/vision` 中的 `torchvision.models.resnet`
- 3Blue1Brown 的 "But what is a neural network?" 系列可帮助建立直觉

**检验标准**：

- 能手写Bottleneck Block，不查任何资料
- 能解释清楚"1x1卷积降维"的参数量对比（64->256通道：不降维时3x3卷积参数量 vs 先降到64再3x3再升回256）

#### 1.2.7 EfficientNet（2019）

**学什么**：

- 核心创新 —— 复合缩放（Compound Scaling）：
  - 之前的网络只在深度（层数）、宽度（通道数）、分辨率中的一个维度上缩放
  - EfficientNet提出同时按固定比例缩放三个维度：$depth = \alpha^\phi, width = \beta^\phi, resolution = \gamma^\phi$，约束 $\alpha \cdot \beta^2 \cdot \gamma^2 \approx 2$
  - 通过网格搜索找到最优的 $\alpha, \beta, \gamma$，然后用 $\phi$ 统一控制模型大小
- 基础网络 EfficientNet-B0 用了神经架构搜索（NAS）找到的 MBConv（Mobile Inverted Bottleneck）作为基本模块
- B0到B7：通过调整 $\phi$ 得到不同规模的模型

**为什么学**：

EfficientNet展示了"如何系统地设计网络规模"的思路。虽然在自动驾驶中用得不如ResNet多，但其设计哲学值得学习。

**学到什么程度**：

- 理解复合缩放的核心思想和三个维度的缩放策略
- 知道MBConv的基本结构（倒残差结构 + SE注意力模块）
- 不需要手写代码，但能用PyTorch调用预训练模型

**学习时间**：半天

**推荐资源**：

- 原论文：Tan & Le, "EfficientNet: Rethinking Model Scaling for Convolutional Neural Networks" (2019)
- 知乎文章：搜索 "EfficientNet 详解"
- GitHub：`lukemelas/EfficientNet-PyTorch`

#### 1.2.8 ConvNeXt（2022）

**学什么**：

- 核心思想：纯卷积网络能否达到和Transformer一样甚至更好的性能？
- 从ResNet出发，逐步"现代化"改造：
  1. 宏观设计：调整各阶段的块数比例（ResNet的3:4:6:3改为1:1:3:1，类似Swin Transformer）
  2. 将Bottleneck改为倒残差结构（中间宽两头窄，类似MobileNet V2）
  3. 大卷积核：用7x7深度可分离卷积替代3x3（更大感受野，对应Transformer的全局注意力）
  4. 微观设计：用LayerNorm替代BatchNorm、GELU替代ReLU、更少的激活函数和归一化层
  5. 独立的下采样层（用步长为2的卷积替代池化）
- 结论：经过这些改造后的ConvNeXt在ImageNet上与Swin Transformer性能相当，证明了很多Transformer的成功因素不是注意力机制独有的

**为什么学**：

ConvNeXt代表了CNN的最新设计理念，也说明了很多架构改进的trick可以跨架构迁移。在自动驾驶中，ConvNeXt可以作为高性能backbone使用。

**学到什么程度**：

- 理解"从ResNet到ConvNeXt"的每一步改造及其动机
- 知道深度可分离卷积、大卷积核、LayerNorm等技术选择的原因
- 能用 `timm` 库调用预训练的ConvNeXt模型

**学习时间**：1天

**推荐资源**：

- 原论文：Liu et al., "A ConvNet for the 2020s" (2022)
- 李沐论文精读视频（B站搜索"李沐 ConvNeXt"）
- GitHub：`facebookresearch/ConvNeXt`

**架构演进整体检验标准**：

- 能按时间顺序列出每个架构的核心创新（一句话总结）
- 能回答"为什么需要更深/更宽/更高效的网络"
- 能用 `torchvision.models` 或 `timm` 加载任意预训练模型并进行推理

---

### 1.3 损失函数

#### 1.3.1 交叉熵损失（Cross-Entropy Loss）

**学什么**：

- 信息论基础：信息量 $I(x) = -\log P(x)$、熵 $H = -\sum P(x)\log P(x)$、KL散度
- 交叉熵的定义：$H(p, q) = -\sum_{i} p_i \log q_i$，其中 $p$ 是真实分布，$q$ 是预测分布
- 二分类交叉熵（Binary Cross-Entropy, BCE）：$L = -[y\log\hat{y} + (1-y)\log(1-\hat{y})]$
- 多分类交叉熵：$L = -\sum_{i=1}^{C} y_i \log(\hat{y}_i)$
- 与softmax结合使用：`nn.CrossEntropyLoss` 内部已包含softmax
- 数值稳定性：为什么要先取log再算loss（防止概率为0时log为负无穷）

**为什么学**：

交叉熵是分类任务中最基础的损失函数，也是目标检测中分类分支的标准损失函数。

**学到什么程度**：

- 能手推二分类和多分类交叉熵的公式
- 能用PyTorch实现 `nn.CrossEntropyLoss` 和 `nn.BCEWithLogitsLoss`
- 理解为什么交叉熵比MSE更适合分类任务（梯度特性）

**学习时间**：半天

**推荐资源**：

- 《动手学深度学习》4.1节 "Softmax回归" 和 4.4节
- 知乎文章：搜索"交叉熵损失函数详解"

#### 1.3.2 Focal Loss（2017）

**学什么**：

- 问题背景：目标检测中正负样本严重不平衡（背景框远多于目标框），简单的交叉熵会被大量易分类的负样本主导
- Focal Loss公式：$FL(p_t) = -\alpha_t (1-p_t)^\gamma \log(p_t)$
  - $(1-p_t)^\gamma$ 是调制因子：当样本被正确分类（$p_t$ 大）时，该因子趋近0，降低其loss贡献
  - $\gamma$（聚焦参数）越大，对易分类样本的抑制越强（通常取2.0）
  - $\alpha$（平衡参数）用于平衡正负样本的权重（通常取0.25）
- 与交叉熵的关系：当 $\gamma=0$ 时，退化为标准交叉熵

**为什么学**：

Focal Loss是RetinaNet的核心贡献，在自动驾驶目标检测中广泛使用（小目标、远距离目标、类别不平衡问题突出）。

**学到什么程度**：

- 能手推Focal Loss公式，理解调制因子的工作原理
- 能用PyTorch自定义实现Focal Loss
- 理解 $\alpha$ 和 $\gamma$ 的作用及其调参策略

**学习时间**：半天

**推荐资源**：

- 原论文：Lin et al., "Focal Loss for Dense Object Detection" (2017)
- GitHub：搜索 "focal loss pytorch"，推荐 `yhenon/pytorch-retinanet`

#### 1.3.3 回归损失

**学什么**：

- L1 Loss（MAE）：$|x - y|$，对异常值鲁棒但梯度恒定
- L2 Loss（MSE）：$(x - y)^2$，对异常值敏感但梯度会随误差减小
- Smooth L1 Loss：结合L1和L2的优点
  - 误差小时用L2（梯度平滑），误差大时用L1（梯度不会爆炸）
  - $SmoothL1(x) = \begin{cases} 0.5x^2 & |x| < 1 \\ |x| - 0.5 & otherwise \end{cases}$
- IoU Loss：直接优化IoU指标，$L_{IoU} = 1 - IoU$
- GIoU Loss：解决IoU Loss在框不重叠时梯度为0的问题
- DIoU Loss：同时考虑距离和重叠面积
- CIoU Loss：在DIoU基础上考虑长宽比一致性

**为什么学**：

目标检测中回归分支（定位框）使用这些损失函数。从L1到CIoU的演进反映了"让损失函数更好地反映我们真正关心的指标（IoU）"的思路。

**学到什么程度**：

- 能实现Smooth L1 Loss和CIoU Loss
- 理解IoU系列损失的演进动机
- 知道什么情况下选哪个损失函数

**学习时间**：1天

**推荐资源**：

- 原论文：IoU(2016), GIoU(2019), DIoU & CIoU(2020)
- 知乎文章：搜索"IoU GIoU DIoU CIoU Loss 详解"

#### 1.3.4 对比学习损失（Contrastive Loss）

**学什么**：

- 核心思想：拉近相似样本的特征表示，推远不相似样本的特征表示
- 对比损失（Contrastive Loss）：$L = y \cdot d^2 + (1-y) \cdot \max(m-d, 0)^2$
- Triplet Loss：$L = \max(d(a,p) - d(a,n) + m, 0)$，需要选择合适的负样本（hard negative mining）
- InfoNCE Loss（SimCLR使用）：$L = -\log \frac{\exp(sim(z_i, z_j)/\tau)}{\sum_{k=1}^{2N} \mathbf{1}_{k \neq i} \exp(sim(z_i, z_k)/\tau)}$
- 温度参数 $\tau$ 的作用

**为什么学**：

对比学习在自动驾驶中用于学习通用的视觉特征表示（自监督预训练），也用于ReID（行人重识别、车辆重识别）等任务。

**学到什么程度**：

- 理解对比学习的基本框架：正样本对、负样本对、特征空间中的距离
- 能实现InfoNCE Loss
- 了解SimCLR、MoCo的基本思路（不需要深入细节）

**学习时间**：1天

**推荐资源**：

- 原论文：SimCLR (Chen et al., 2020), MoCo (He et al., 2020)
- 李沐论文精读："SimCLR论文精读"（B站）
- 知乎文章：搜索"对比学习入门"

---

### 1.4 优化器

#### 1.4.1 SGD及其变体

**学什么**：

- 基本SGD：$\theta_{t+1} = \theta_t - \eta \nabla L(\theta_t)$
- 动量SGD（Momentum）：$\theta_{t+1} = \theta_t - \eta v_t$，其中 $v_t = \mu v_{t-1} + \nabla L$
  - 直觉理解：用控制理论的话说，就是给梯度更新加了一个"惯性"，减少震荡
  - $\mu$（动量系数）通常取0.9
- Nesterov动量：先"预看"一步再计算梯度，收敛更快

**为什么学**：

SGD虽然收敛慢，但在很多视觉任务（特别是检测、分割）上泛化性优于Adam。YOLO等检测框架通常默认使用SGD+Momentum。

**学到什么程度**：

- 能从控制系统的角度理解动量的作用（阻尼振荡）
- 能用PyTorch的 `torch.optim.SGD` 并理解momentum参数的含义

**学习时间**：半天

**推荐资源**：

- 《深度学习》第8章"优化"
- Sebastian Ruder 的博客："An overview of gradient descent optimization algorithms"（https://ruder.io/optimizing-gradient-descent/）—— 经典综述

#### 1.4.2 Adam / AdamW

**学什么**：

- Adam（Adaptive Moment Estimation）：
  - 结合了动量和自适应学习率
  - 一阶矩估计（梯度均值）：$m_t = \beta_1 m_{t-1} + (1-\beta_1) g_t$
  - 二阶矩估计（梯度方差）：$v_t = \beta_2 v_{t-1} + (1-\beta_2) g_t^2$
  - 偏差修正：$\hat{m}_t = m_t / (1 - \beta_1^t)$, $\hat{v}_t = v_t / (1 - \beta_2^t)$
  - 更新：$\theta_{t+1} = \theta_t - \eta \cdot \hat{m}_t / (\sqrt{\hat{v}_t} + \epsilon)$
  - 默认参数：$\beta_1=0.9, \beta_2=0.999, \epsilon=10^{-8}$
- AdamW（Weight Decay正确的Adam）：
  - 原始Adam中L2正则化和权重衰减不等价
  - AdamW直接在参数更新中加权重衰减：$\theta_{t+1} = \theta_t - \eta(\hat{m}_t / (\sqrt{\hat{v}_t} + \epsilon) + \lambda \theta_t)$
  - Transformer模型通常使用AdamW

**为什么学**：

Adam/AdamW是目前最常用的优化器。YOLO系列早期版本用SGD，后续版本（如YOLOv8的Ultralytics实现）开始支持AdamW。Transformer类模型（DETR等）几乎都用AdamW。

**学到什么程度**：

- 能手推Adam的更新公式
- 能解释"为什么Adam比SGD收敛快但泛化性可能差"
- 能解释AdamW与Adam+L2正则的区别
- 能用 `torch.optim.AdamW` 并调参

**学习时间**：1天

**推荐资源**：

- 原论文：Kingma & Ba, "Adam: A Method for Stochastic Optimization" (2014)
- Loshchilov & Hutter, "Decoupled Weight Decay Regularization" (2019)（AdamW）
- 上述Ruder的博客

#### 1.4.3 学习率调度策略

**学什么**：

- 为什么需要学习率调度：训练初期需要大步探索，后期需要小步微调
- 常见调度策略：
  - StepLR：每隔固定epoch将学习率乘以一个因子（如每30个epoch乘以0.1）
  - MultiStepLR：在指定的milestone处衰减
  - ExponentialLR：指数衰减
  - CosineAnnealingLR：学习率按余弦函数从初始值降到最小值（目前最主流）
  - OneCycleLR：先升后降，结合warmup
  - ReduceOnPlateau：验证集指标不再提升时衰减

**为什么学**：

学习率调度对训练效果影响巨大，选错调度策略可能导致模型不收敛或泛化性差。在实际项目中，Cosine退火是目前最常用的策略。

**学到什么程度**：

- 能用PyTorch的 `torch.optim.lr_scheduler` 实现各种调度策略
- 理解Cosine退火的公式和曲线形状
- 能画出不同调度策略的学习率曲线对比图

**学习时间**：半天

**推荐资源**：

- 《动手学深度学习》12.10节 "学习率调度"
- PyTorch官方文档：`torch.optim.lr_scheduler`

---

### 1.5 正则化

#### 1.5.1 Dropout

**学什么**：

- 原理：训练时以概率 $p$ 随机将神经元输出置零，测试时使用全部神经元但输出乘以 $1-p$
- 数学直觉：相当于训练了一个"子网络集成"（每次Dropout得到一个不同的子网络）
- 为什么有效：减少神经元之间的共适应（co-adaptation），增强模型的鲁棒性
- 常用 $p$ 值：0.5（全连接层）、0.1（卷积层较少使用）

**为什么学**：

Dropout是经典的正则化手段。在目标检测中，全连接头（如Faster R-CNN的分类头）中使用Dropout。

**学到什么程度**：

- 能解释Dropout在训练和测试时的行为差异
- 能用 `nn.Dropout` 实现
- 理解为什么现在主流的CNN/Transformer架构中Dropout用得少了（被BatchNorm/LayerNorm取代）

**学习时间**：2小时

#### 1.5.2 Batch Normalization（BN）

**学什么**：

- 问题：深层网络中每一层输入的分布随训练变化（内部协变量偏移，Internal Covariate Shift）
- BN操作：
  1. 对mini-batch计算均值 $\mu_B$ 和方差 $\sigma_B^2$
  2. 归一化：$\hat{x} = (x - \mu_B) / \sqrt{\sigma_B^2 + \epsilon}$
  3. 缩放和偏移：$y = \gamma \hat{x} + \beta$（$\gamma$ 和 $\beta$ 是可学习参数）
- 训练时用batch统计量，测试时用训练过程中积累的running mean和running variance
- BN放在卷积层之后、激活函数之前（或之后，不同实现有差异）
- BN的副作用：需要足够的batch size（自动驾驶中高分辨率图像导致batch size通常很小）

**为什么学**：

BN是CNN中最重要的正则化和训练稳定化技术之一。几乎所有主流CNN架构都使用BN。在自动驾驶中，由于输入图像分辨率大，batch size受限，此时需要考虑Group Norm等替代方案。

**学到什么程度**：

- 能手推BN的前向和反向传播公式
- 能解释训练和测试时BN的行为差异（为什么测试时用running statistics）
- 能实现 `nn.BatchNorm2d` 并理解其参数
- 了解GN（Group Normalization）作为小batch替代方案

**学习时间**：半天

**推荐资源**：

- 原论文：Ioffe & Szegedy, "Batch Normalization: Accelerating Deep Network Training" (2015)
- 《动手学深度学习》7.5节 "批量归一化"
- 知乎文章：搜索"Batch Normalization 原理详解"

#### 1.5.3 Layer Normalization（LN）

**学什么**：

- 与BN的区别：BN在batch维度归一化，LN在channel维度归一化（对每个样本独立归一化）
- LN的计算：对一个样本的所有通道计算均值和方差，然后归一化
- 优势：不依赖batch size，适合batch size很小或动态变化的场景
- 主要用于Transformer架构（Self-Attention + FFN之后加LN）
- Pre-LN vs Post-LN的区别（现代Transformer常用Pre-LN）

**为什么学**：

LN是Vision Transformer的核心组件。在自动驾驶中使用DETR等Transformer检测器时，LN是标准配置。

**学到什么程度**：

- 能用 `nn.LayerNorm` 实现，理解 `normalized_shape` 参数的含义
- 能清晰对比BN和LN的区别（归一化维度、适用场景）

**学习时间**：2小时

#### 1.5.4 权重衰减（Weight Decay）

**学什么**：

- L2正则化：在损失函数中加 $\frac{\lambda}{2}||\theta||_2^2$
- 权重衰减：$\theta_{t+1} = (1 - \lambda\eta)\theta_t - \eta\nabla L$
- 在SGD中，L2正则化和权重衰减等价；在Adam中不等价（所以需要AdamW）
- 实践中的 $\lambda$ 选择：通常 $10^{-4}$ 到 $10^{-2}$

**为什么学**：

权重衰减是最基本的正则化手段，几乎所有训练都会用到。理解它和L2正则的区别对正确使用AdamW很重要。

**学到什么程度**：

- 理解权重衰减的数学原理及其与L2正则的区别
- 能在PyTorch中正确设置 `weight_decay` 参数

**学习时间**：2小时

---

### 1.6 训练技巧

#### 1.6.1 Warmup

**学什么**：

- 问题：训练刚开始时，模型参数是随机初始化的，如果直接用大学习率可能导致训练不稳定
- Warmup策略：在前N个epoch从很小的学习率线性增长到目标学习率
- 常见设置：前5-10个epoch做warmup
- 为什么有效：模型初始化时梯度方向不可靠，用小学习率"试探"方向后再加大学习率

**为什么学**：

Warmup几乎是所有深度学习训练的标准配置。YOLO系列、DETR等检测模型都默认使用warmup。

**学到什么程度**：

- 能用PyTorch实现线性warmup（手动修改学习率或使用scheduler）
- 知道warmup epoch数的典型设置

**学习时间**：1小时

#### 1.6.2 Cosine退火（Cosine Annealing）

**学什么**：

- 学习率按余弦函数从 $\eta_{max}$ 衰减到 $\eta_{min}$：
  $\eta_t = \eta_{min} + \frac{1}{2}(\eta_{max} - \eta_{min})(1 + \cos(\frac{t}{T}\pi))$
- Warmup + Cosine退火的组合：先线性warmup，再cosine衰减，是目前最主流的策略
- Cosine Annealing with Warm Restarts（SGDR）：周期性重启学习率

**为什么学**：

Warmup + Cosine退火是目前检测模型训练的"标配"组合。

**学到什么程度**：

- 能画出cosine退火的学习率曲线
- 能用PyTorch的 `CosineAnnealingLR` 或手动实现

**学习时间**：1小时

#### 1.6.3 EMA（Exponential Moving Average）

**学什么**：

- 原理：维护模型参数的指数移动平均版本：$\theta_{ema} = \alpha \cdot \theta_{ema} + (1-\alpha) \cdot \theta$
  - $\alpha$ 通常取0.9999或0.999
- 作用：EMA模型比训练中的实时参数更稳定，通常用于最终推理
- 在YOLOv8中默认开启EMA

**为什么学**：

EMA是提升模型最终性能的简单有效trick，在自动驾驶检测模型训练中广泛使用。

**学到什么程度**：

- 理解EMA的原理和使用方式
- 能实现一个EMA类，在训练循环中更新EMA参数
- 知道 $\alpha$ 的典型取值

**学习时间**：1-2小时

**推荐资源**：

- GitHub：搜索"model ema pytorch"，有很多现成实现

#### 1.6.4 混合精度训练（AMP）

**学什么**：

- FP32 vs FP16 vs BF16：不同浮点精度的范围和精度
- 混合精度训练原理：
  - 用FP16做前向和反向传播（加速计算、减少显存）
  - 用FP32维护主权重副本（防止精度损失）
  - Loss Scaling：将loss乘以一个大的数（如1024），防止FP16下梯度下溢（underflow），更新参数时再除回去
- PyTorch中的实现：`torch.cuda.amp.GradScaler` + `torch.cuda.amp.autocast`

**为什么学**：

自动驾驶中图像分辨率高、模型大，显存和训练速度是关键瓶颈。AMP可以在几乎不损失精度的情况下训练速度提升1.5-2倍，显存减少约30%。YOLO系列默认开启AMP。

**学到什么程度**：

- 能用PyTorch的 `torch.cuda.amp` 实现混合精度训练
- 理解为什么需要Loss Scaling
- 知道FP16和BF16的区别

**学习时间**：2-3小时

**推荐资源**：

- PyTorch官方教程："Automatic Mixed Precision"（https://pytorch.org/tutorials/recipes/recipes/amp_recipe.html）
- NVIDIA博客：搜索"mixed precision training"

---

### 1.7 迁移学习与预训练模型

**学什么**：

- 迁移学习的核心思想：在大规模数据集（如ImageNet）上预训练的模型，其特征提取能力可以迁移到其他任务
- 三种迁移策略：
  1. 特征提取（Feature Extraction）：冻结预训练backbone，只训练新添加的分类头
  2. 微调（Fine-tuning）：解冻部分或全部层，用小学习率继续训练
  3. 从头训练（Training from Scratch）：不使用预训练权重
- 何时用哪种策略：
  - 目标数据集小且与ImageNet相似 -> 特征提取
  - 目标数据集大或与ImageNet差异大 -> 微调或从头训练
- ImageNet预训练权重的重要性：几乎所有自动驾驶检测模型都使用ImageNet预训练的backbone
- 常用预训练模型来源：`torchvision.models`、`timm` 库（https://github.com/huggingface/pytorch-image-models）

**为什么学**：

迁移学习是实际项目中的核心技能。自动驾驶标注数据昂贵，迁移学习能大幅减少所需数据量和训练时间。

**学到什么程度**：

- 能用PyTorch加载预训练模型并修改分类头
- 能实现冻结/解冻部分层的微调策略
- 熟悉 `timm` 库的使用：`timm.create_model('resnet50', pretrained=True, num_classes=10)`
- 了解"backbone + head"的模型设计范式

**学习时间**：1天

**推荐资源**：

- PyTorch官方教程："Finetuning Torchvision Models"（https://pytorch.org/tutorials/beginner/finetuning_torchvision_models_tutorial.html）
- `timm` 库文档和示例
- 《动手学深度学习》迁移学习章节

---

### 1.8 PyTorch完全掌握

#### 1.8.1 Tensor操作

**学什么**：

- Tensor创建：`torch.tensor()`, `torch.zeros()`, `torch.randn()`, `torch.arange()`
- 基本操作：加减乘除、矩阵乘法（`torch.mm`, `torch.matmul`, `@`）
- 形状操作：`view/reshape`, `permute/transpose`, `squeeze/unsqueeze`, `cat/stack`
- 索引和切片：与NumPy类似但有GPU支持
- 设备管理：`.to(device)`, `cuda()`, `cpu()`
- 自动求导相关：`.requires_grad_()`, `.detach()`, `.item()`

**为什么学**：

Tensor是PyTorch的基本数据结构，所有模型输入、输出、参数都是Tensor。熟练的Tensor操作能力是高效开发的基础。

**学到什么程度**：

- 能不查文档完成常见的Tensor操作
- 能进行复杂的维度变换（检测任务中经常需要处理不同形状的tensor）
- 理解view/reshape的区别（contiguous问题）

**学习时间**：2-3天

**推荐资源**：

- PyTorch官方教程："Introduction to PyTorch"（https://pytorch.org/tutorials/beginner/basics/intro.html）
- 《动手学深度学习》PyTorch部分
- B站搜索"PyTorch入门"系列教程

#### 1.8.2 自动求导机制（Autograd）

**学什么**：

- 计算图（Computational Graph）：PyTorch构建动态计算图（Dynamic Computational Graph）
- `torch.autograd.grad()` 和 `.backward()` 的工作原理
- 叶子节点（leaf tensor）和非叶子节点的区别
- `with torch.no_grad()` 的作用：推理时关闭梯度计算，节省显存和计算
- 梯度累积（Gradient Accumulation）：当batch size太大时，分几个小batch计算梯度再累积
- `loss.backward(retain_graph=True)` 的使用场景

**为什么学**：

理解Autograd机制是排查训练问题（如梯度消失、内存泄漏）的基础。梯度累积在自动驾驶中很常用（大图导致batch size小）。

**学到什么程度**：

- 能用 `torch.autograd.grad` 手动计算指定变量的梯度
- 能实现梯度累积的训练循环
- 能调试"RuntimeError: Trying to backward through the graph a second time"等常见错误

**学习时间**：1-2天

#### 1.8.3 Dataset / DataLoader

**学什么**：

- `torch.utils.data.Dataset`：自定义数据集需要实现 `__len__` 和 `__getitem__`
- `torch.utils.data.DataLoader`：
  - `batch_size`、`shuffle`、`num_workers`、`pin_memory`、`drop_last`
  - 自动批处理、多进程数据加载
  - `collate_fn`：自定义批处理逻辑（检测任务中每张图的box数量不同，需要自定义collate）
- `torchvision.transforms`：图像预处理流水线（Resize, ToTensor, Normalize, RandomHorizontalFlip等）
- `albumentations` 库：更强的数据增强库，支持检测框的同步变换

**为什么学**：

数据处理是工程落地中最耗时的环节之一。检测任务的数据集比分类复杂得多（图像+标注框+类别），自定义Dataset和collate_fn是必须掌握的技能。

**学到什么程度**：

- 能从零实现一个目标检测的Dataset（读取图像和标注文件，返回image和boxes/labels）
- 能自定义collate_fn处理变长的检测标注
- 能使用 `albumentations` 实现数据增强
- 能调通num_workers、pin_memory等参数优化数据加载速度

**学习时间**：2-3天

**推荐资源**：

- PyTorch官方教程："Datasets & DataLoaders"（https://pytorch.org/tutorials/beginner/basics/data_tutorial.html）
- `albumentations` 库文档：https://albumentations.ai/
- COCO数据集的PyTorch加载实现（`torchvision.datasets.CocoDetection`）

#### 1.8.4 自定义模型

**学什么**：

- `torch.nn.Module`：所有模型的基类
  - `__init__`：定义网络层
  - `forward`：定义前向传播逻辑
- 常用层：`nn.Conv2d`, `nn.BatchNorm2d`, `nn.ReLU`, `nn.Linear`, `nn.MaxPool2d`
- 容器：`nn.Sequential`, `nn.ModuleList`, `nn.ModuleDict`
- 参数管理：`model.parameters()`, `model.named_parameters()`, `model.state_dict()`, `model.load_state_dict()`
- 模型保存和加载：`torch.save()`, `torch.load()`, `torch.load_state_dict()`

**为什么学**：

能自定义模型是实现检测器、修改backbone、添加新模块的基础。

**学到什么程度**：

- 能从零写一个完整的CNN分类模型
- 能用 `nn.ModuleList` 实现多层结构
- 能正确实现模型保存、加载、迁移学习（strict=False加载部分权重）

**学习时间**：2天

#### 1.8.5 训练循环

**学什么**：

- 标准训练循环的完整流程：
  ```python
  for epoch in range(num_epochs):
      model.train()
      for images, labels in train_loader:
          images, labels = images.to(device), labels.to(device)
          optimizer.zero_grad()
          outputs = model(images)
          loss = criterion(outputs, labels)
          loss.backward()
          optimizer.step()
      # 验证
      model.eval()
      with torch.no_grad():
          for images, labels in val_loader:
              ...
      scheduler.step()
  ```
- `model.train()` vs `model.eval()` 的区别（影响BN和Dropout的行为）
- 梯度裁剪（Gradient Clipping）：`torch.nn.utils.clip_grad_norm_`
- TensorBoard / Wandb 训练监控

**为什么学**：

训练循环是深度学习工程的核心代码，需要写得规范、高效、可调试。

**学到什么程度**：

- 能不看参考代码写出完整的训练循环（包含train/val/scheduler/save/checkpoint）
- 能实现梯度裁剪、学习率warmup、模型保存/恢复
- 能使用TensorBoard或Wandb记录训练过程

**学习时间**：1-2天

#### 1.8.6 分布式训练（DDP基础）

**学什么**：

- 为什么需要分布式训练：自动驾驶数据量大、模型大，单GPU训练太慢
- PyTorch DDP（DistributedDataParallel）基本原理：
  - 每个GPU一个进程，各自持有模型副本
  - 每个进程使用DistributedSampler获取不重叠的数据子集
  - 反向传播后自动同步梯度（AllReduce操作）
- 关键API：
  - `torch.distributed.init_process_group`
  - `torch.nn.parallel.DistributedDataParallel`
  - `torch.utils.data.distributed.DistributedSampler`
- 启动方式：`torch.distributed.launch` 或 `torchrun`

**为什么学**：

实际项目中多GPU训练是常态。YOLO系列的训练脚本都支持DDP。

**学到什么程度**：

- 能把一个单GPU训练脚本改造为DDP多GPU训练
- 理解DistributedSampler的作用
- 知道DDP和DataParallel（DP）的区别（DDP用多进程，DP用多线程，DDP效率高得多）

**学习时间**：1-2天

**推荐资源**：

- PyTorch官方教程："Getting Started with Distributed Data Parallel"（https://pytorch.org/tutorials/intermediate/ddp_tutorial.html）
- 知乎文章：搜索"PyTorch DDP 分布式训练"

---

## Part 2: 目标检测（2D）- 自动驾驶感知核心

### 2.1 检测基础概念

#### 2.1.1 Bounding Box

**学什么**：

- 边界框的两种表示方式：
  - `(x1, y1, x2, y2)`：左上角和右下角坐标
  - `(cx, cy, w, h)`：中心点坐标和宽高
- 坐标系：图像坐标系（y轴向下）
- 不同数据集的标注格式：COCO（`[x, y, w, h]`）、VOC（`[x1, y1, x2, y2]`）、YOLO格式（归一化的`[cx, cy, w, h]`）
- 格式之间的转换

**为什么学**：

这是检测任务的基本语言。不同数据集、不同框架使用不同的标注格式，格式转换是工程中的常见操作。

**学到什么程度**：

- 能熟练进行各种bbox格式之间的转换
- 能在图像上绘制检测框（用OpenCV或matplotlib）

**学习时间**：2小时

#### 2.1.2 IoU（Intersection over Union）

**学什么**：

- IoU定义：$IoU = \frac{|A \cap B|}{|A \cup B|} = \frac{交集面积}{并集面积}$
- 计算方法：先算交集区域的坐标，再算面积
- IoU的取值范围：[0, 1]，1表示完全重叠
- 扩展形式：GIoU、DIoU、CIoU（在损失函数部分已介绍）

**为什么学**：

IoU是衡量检测框质量的核心指标，也是NMS和评估指标的基础。

**学到什么程度**：

- 能手写IoU计算函数（包括batch版本的高效实现）
- 能画图理解IoU的几何含义

**学习时间**：1小时

#### 2.1.3 NMS（Non-Maximum Suppression）

**学什么**：

- 问题：一个目标可能产生多个重叠的检测框，需要去重
- NMS算法步骤：
  1. 按置信度分数降序排列所有框
  2. 取分数最高的框加入结果集
  3. 删除与该框IoU > 阈值的所有框
  4. 重复步骤2-3直到所有框都被处理
- Soft-NMS：不直接删除重叠框，而是降低其置信度分数
- NMS的超参数：IoU阈值（通常0.5或0.6）

**为什么学**：

NMS是所有检测器后处理的标准步骤。在自动驾驶中，密集场景下NMS的性能直接影响检测效果。

**学到什么程度**：

- 能手写NMS算法（纯Python和PyTorch版本）
- 能用 `torchvision.ops.nms` 或 `torchvision.ops.batched_nms`
- 理解Soft-NMS的原理和适用场景

**学习时间**：半天

**推荐资源**：

- 知乎文章：搜索"NMS 非极大值抑制 详解"
- GitHub：`torchvision.ops` 中的NMS实现

#### 2.1.4 mAP（mean Average Precision）

**学什么**：

- Precision和Recall的定义：
  - $Precision = \frac{TP}{TP + FP}$（检测出的框中正确的比例）
  - $Recall = \frac{TP}{TP + FN}$（所有真实目标中被检测到的比例）
- 置信度阈值的影响：阈值越高，Precision越高但Recall越低
- PR曲线（Precision-Recall Curve）：不同置信度阈值下的Precision-Recall对
- AP（Average Precision）：PR曲线下的面积
  - COCO的AP计算：在多个IoU阈值（0.5:0.05:0.95）下计算AP的平均值
- mAP：所有类别的AP平均值
- COCO评估指标：
  - AP@0.5（VOC风格）
  - AP@[0.5:0.95]（COCO标准，最常用）
  - AP_S, AP_M, AP_L（小/中/大目标的AP）

**为什么学**：

mAP是检测模型的核心评估指标。理解它的计算过程有助于分析模型的优缺点（如"模型对小目标检测差"需要看AP_S）。

**学到什么程度**：

- 能手写AP计算代码（VOC风格和COCO风格）
- 能用 `pycocotools` 对COCO格式的检测结果进行评估
- 能分析检测结果的各类AP，诊断模型问题

**学习时间**：1天

**推荐资源**：

- COCO官方评估代码：https://github.com/cocodataset/cocoapi
- 知乎文章：搜索"目标检测 mAP 计算详解"

---

### 2.2 Two-stage方法

#### 2.2.1 R-CNN（2014）→ Fast R-CNN（2015）→ Faster R-CNN（2015）

**学什么**：

**R-CNN（Region-based CNN）**：
- 流程：Selective Search提取约2000个候选区域 -> 每个区域用CNN提特征 -> SVM分类 + 线性回归修正框
- 问题：每个候选区域都要过CNN，速度极慢（一张图47秒）

**Fast R-CNN**：
- 改进：整张图过一次CNN得到特征图，然后在特征图上提取各候选区域的特征（RoI Pooling）
- RoI Pooling：将不同大小的RoI映射为固定大小的特征（如7x7）
- 多任务损失：分类损失 + 边框回归损失，联合训练
- 问题：候选区域生成仍用Selective Search，成为瓶颈

**Faster R-CNN**（重点）：
- **核心创新 —— RPN（Region Proposal Network）**：
  - 用神经网络替代Selective Search生成候选区域
  - 在特征图的每个位置设置k个anchor（不同尺度和长宽比）
  - RPN输出：每个anchor是否包含目标的分数 + 边框修正量
  - 训练：正样本（IoU>0.7）和负样本（IoU<0.3）采样
- 整体流程：CNN Backbone提特征 -> RPN生成候选区域 -> RoI Pooling -> 分类 + 回归头
- "两阶段"的含义：第一阶段（RPN）做候选区域筛选，第二阶段做精细分类和定位

**为什么学**：

Faster R-CNN是目标检测的里程碑，RPN的设计思想影响了后续几乎所有检测器。理解Faster R-CNN是理解所有two-stage方法的基础。在自动驾驶中，Faster R-CNN仍被用作baseline。

**学到什么程度**：

- 能画出Faster R-CNN的完整流程图（Backbone -> RPN -> RoI Pooling -> Head）
- 能解释RPN中anchor的设计和正负样本分配策略
- 能解释RoI Pooling和RoI Align的区别（RoI Align解决了量化误差问题）
- 能用 `torchvision.models.detection.fasterrcnn_resnet50_fpn` 进行推理和微调

**学习时间**：3-4天

**推荐资源**：

- 原论文：
  - Girshick et al., "Rich feature hierarchies for accurate object detection and semantic segmentation" (R-CNN, 2014)
  - Girshick, "Fast R-CNN" (2015)
  - Ren et al., "Faster R-CNN: Towards Real-Time Object Detection with Region Proposal Networks" (2015)
- 李沐论文精读："Faster R-CNN论文精读"（B站）—— 强烈推荐
- 知乎文章：搜索"Faster R-CNN 详解"
- GitHub：`pytorch/vision` 中的 `torchvision.models.detection`

---

### 2.3 One-stage方法

#### 2.3.1 SSD（Single Shot MultiBox Detector, 2016）

**学什么**：

- 核心思想：一步完成候选区域生成和分类，不需要RPN
- 多尺度特征图检测：在不同分辨率的特征图上检测不同大小的目标
  - 浅层特征图（大分辨率）检测小目标
  - 深层特征图（小分辨率）检测大目标
- Default boxes（类似于anchor）：在每个特征图位置设置不同比例的框
- 使用VGG-16作为backbone，后面接额外的卷积层

**为什么学**：

SSD是第一个真正实用的one-stage检测器，多尺度检测的思想被后续所有检测器继承。

**学到什么程度**：

- 理解多尺度检测的原理（为什么需要在不同层检测）
- 知道SSD和Faster R-CNN的核心区别

**学习时间**：半天

**推荐资源**：

- 原论文：Liu et al., "SSD: Single Shot MultiBox Detector" (2016)
- 知乎文章：搜索"SSD 目标检测 详解"

#### 2.3.2 YOLO系列演进

**YOLOv1（2016）**

**学什么**：

- 核心思想："You Only Look Once"——将检测问题转化为回归问题
- 将图像划分为 S×S 网格，每个网格预测 B 个边界框和 C 个类别概率
- 每个边界框预测5个值：(x, y, w, h, confidence)
- 缺点：每个网格只能检测一个目标，对密集小目标效果差

**学到什么程度**：了解YOLO的基本思想即可，不需要深入细节。

**学习时间**：2小时

---

**YOLOv2 / YOLO9000（2017）**

**学什么**：

- Batch Normalization：所有卷积层后加BN
- 高分辨率分类器：先在448x448上微调分类网络
- 使用anchor boxes（借鉴Faster R-CNN的RPN思想）
- Dimension Clusters：用K-means聚类找到适合数据集的anchor尺寸
- Passthrough层：将浅层特征引入深层（类似后来的FPN思想）
- 多尺度训练：每隔几个batch改变输入分辨率

**学到什么程度**：了解YOLOv2的关键改进点。

**学习时间**：2小时

---

**YOLOv3（2018）—— 多尺度检测**

**学什么**：

- **核心创新 —— 多尺度预测（FPN结构）**：
  - 三个尺度的特征图（stride 8/16/32）分别检测小/中/大目标
  - 每个尺度用3个anchor（共9个anchor，通过K-means聚类得到）
- Darknet-53 backbone：引入残差连接（借鉴ResNet）
- 用logistic回归替代softmax做多标签分类（一个目标可以属于多个类别）

**为什么学**：

YOLOv3是YOLO系列中影响力最大的版本之一，其多尺度检测设计奠定了后续YOLO架构的基础。

**学到什么程度**：

- 能画出YOLOv3的网络结构图（backbone + FPN neck + 3个检测头）
- 理解三个尺度分别对应什么大小的目标
- 能用Darknet或PyTorch复现

**学习时间**：1天

**推荐资源**：

- 原论文：Redmon & Farhadi, "YOLOv3: An Incremental Improvement" (2018)
- Joseph Redmon的官方实现：`pjreddie/darknet`
- PyTorch复现：`ultralytics/yolov3`（早期版本）、`Tianxiaomo/pytorch-YOLOv4`
- 知乎文章：搜索"YOLOv3 详解"

---

**YOLOv4（2020）**

**学什么**：

- Bag of Freebies（不影响推理速度的训练技巧）：
  - **Mosaic数据增强**：将4张图拼成一张，增加目标多样性
  - DropBlock正则化
  - CIoU Loss
  - Label Smoothing
- Bag of Specials（略微增加推理速度但提升精度的模块）：
  - **CSPDarknet53** backbone：Cross Stage Partial连接，减少计算量
  - **SPP（Spatial Pyramid Pooling）**：多尺度池化，增大感受野
  - **PANet（Path Aggregation Network）**：双向FPN，增强特征融合
  - Mish激活函数
  - SAM（空间注意力）和CBAM

**为什么学**：

YOLOv4系统性地总结了"哪些trick对检测有效"，其数据增强（Mosaic）和网络结构（CSP+SPP+PANet）设计对后续版本影响深远。

**学到什么程度**：

- 理解CSP结构的设计思想（将特征图分为两部分，一部分直接传递，一部分经过卷积）
- 理解SPP的作用（多尺度感受野融合）
- 理解PANet相比FPN的改进（增加了bottom-up路径）
- 能用 `AlexeyAB/darknet` 或PyTorch复现

**学习时间**：1-2天

**推荐资源**：

- 原论文：Bochkovskiy et al., "YOLOv4: Optimal Speed and Accuracy of Object Detection" (2020)
- GitHub：`WongKinYiu/PyTorch_YOLOv4`、`Tianxiaomo/pytorch-YOLOv4`
- 李沐论文精读（B站搜索"李沐 YOLOv4"）

---

**YOLOv5（2020）**

**学什么**：

- Ultralytics公司开发（非原作者），以工程实现优秀著称
- 基于PyTorch，开箱即用，文档完善
- 模型规模：YOLOv5n/s/m/l/x（从轻量到重量级）
- 核心改进：
  - 自适应anchor计算
  - 自适应图像缩放
  - CSP结构 + SPP + PANet
  - 训练策略：Warmup + Cosine退火、EMA、Mosaic + MixUp增强
- 易用性极高：`pip install ultralytics` 即可使用

**为什么学**：

YOLOv5是实际项目中使用最广泛的YOLO版本之一。它的工程实现非常成熟，适合作为入门实践的起点。

**学到什么程度**：

- 能用YOLOv5训练自定义数据集
- 能理解其目录结构和配置文件（yaml）
- 能进行模型导出（ONNX、TensorRT）

**学习时间**：1-2天

**推荐资源**：

- GitHub：`ultralytics/yolov5`
- 官方文档：https://docs.ultralytics.com/
- B站搜索"YOLOv5 训练自定义数据集"

---

**YOLOv8（2023）—— Anchor-free, 解耦头**

**学什么**：

- **核心创新**：
  - **Anchor-free设计**：不再预设anchor，直接预测中心点偏移和宽高
  - **解耦头（Decoupled Head）**：分类和回归使用独立的分支（之前YOLO用耦合头，一个卷积同时输出分类和回归）
  - C2f模块替代C3模块：更多的跨层连接
  - Task-Aligned Assigner：动态正负样本分配策略（同时考虑分类分数和IoU质量）
  - Distribution Focal Loss：用分布回归替代直接回归坐标值
- Ultralytics统一框架：同一框架支持检测、分割、姿态估计、分类、OBB（旋转目标检测）

**为什么学**：

YOLOv8是目前工程落地中使用最多的检测器，Ultralytics框架极为成熟。后续的YOLOv9-v11也基于类似的框架。

**学到什么程度**：

- 能用Ultralytics框架训练、验证、推理、导出模型
- 能理解anchor-free和anchor-based的区别
- 能理解解耦头的优势（分类和回归的梯度不互相干扰）
- 能自定义数据集并训练YOLOv8

**学习时间**：2-3天

**推荐资源**：

- GitHub：`ultralytics/ultralytics`（官方仓库，支持v8）
- 官方文档：https://docs.ultralytics.com/
- B站搜索"YOLOv8 使用教程"或"YOLOv8 训练自定义数据集"
- 论文：Jocher et al., "Ultralytics YOLOv8" (2023)

---

**YOLOv9 / v10 / v11 最新进展**

**学什么**：

**YOLOv9（2024）**：
- 核心创新：PGI（Programmable Gradient Information）和 GELAN 架构
- 解决信息瓶颈问题：深层网络中信息逐步丢失，PGI通过辅助可逆分支保留完整梯度信息
- GELAN：结合CSPNet和ELAN的高效网络结构

**YOLOv10（2024）**：
- 核心创新：
  - NMS-free：通过一致的双标签分配（Consistent Dual Assignments）消除NMS后处理
  - 效率优化：轻量化分类头、空间-通道解耦下采样
  - 大核卷积和部分自注意力

**YOLO11 / v11（2024-2025, Ultralytics）**：
- Ultralytics发布的最新版本，统一在 `ultralytics` 包中
- 改进的backbone和neck结构
- 支持更多任务（检测、分割、姿态、OBB、分类）
- 效率和精度进一步提升

**为什么学**：

了解最新进展有助于选择合适的模型。NMS-free的设计思路代表了未来方向。

**学到什么程度**：

- 了解各版本的核心创新点（不需要深入所有细节）
- 能用最新的Ultralytics包训练和部署这些模型
- 知道如何根据任务需求选择合适的YOLO版本

**学习时间**：1-2天（快速浏览）

**推荐资源**：

- YOLOv9论文：Wang et al., "YOLOv9: Learning What You Want to Learn Using Programmable Gradient Information" (2024)
- YOLOv10论文：Wang et al., "YOLOv10: Real-Time End-to-End Object Detection" (2024)
- GitHub：`ultralytics/ultralytics`（包含最新版本）
- 知乎/微信公众号：搜索"YOLO系列最新进展"

---

### 2.4 Anchor-based vs Anchor-free

#### 2.4.1 Anchor-based检测器的问题

**学什么**：

- Anchor-based（如Faster R-CNN、YOLOv3-v5）需要预设大量anchor
- 问题：anchor的数量、大小、长宽比需要手动设计，不同数据集需要不同的配置
- 正负样本分配依赖IoU阈值，不够灵活
- 正负样本极度不平衡（大量anchor是负样本）

#### 2.4.2 FCOS（Fully Convolutional One-Stage, 2019）

**学什么**：

- 核心思想：在每个特征图位置直接预测该点到边界框四条边的距离
- 无需anchor，无需NMS（虽然实际上还是需要NMS）
- Center-ness分支：预测每个点是否靠近目标中心，用于抑制低质量检测
- 多尺度预测：FPN的多个level分别负责不同大小的目标

**为什么学**：

FCOS是anchor-free检测的经典代表，其"逐像素预测"的思想对后续CenterNet等方法有重要影响。

**学到什么程度**：

- 能理解FCOS的预测方式（每个位置预测l, t, r, b四个距离）
- 能画出FCOS的正负样本分配示意图
- 理解center-ness分支的作用

**学习时间**：半天

**推荐资源**：

- 原论文：Tian et al., "FCOS: Fully Convolutional One-Stage Object Detection" (2019)
- GitHub：`tianzhi0549/FCOS`

#### 2.4.3 CenterNet（2019）

**学什么**：

- 核心思想：将目标检测为关键点（目标中心点），然后回归宽高
- 输出三个分支：
  1. 热力图（Heatmap）：预测目标中心点位置
  2. 宽高（Size）：预测每个目标的宽度和高度
  3. 偏移（Offset）：预测中心点的亚像素偏移
- 无需NMS：热力图的极大值天然去重
- 非常优雅的框架：一个关键点检测器即可完成检测

**为什么学**：

CenterNet的思想影响了很多后续工作，包括3D检测中的CenterPoint。其"关键点检测"思路也用于车道线检测。

**学到什么程度**：

- 理解将检测转化为关键点检测的思想
- 能画出CenterNet的网络输出结构图
- 理解为什么不需要NMS

**学习时间**：半天

**推荐资源**：

- 原论文：Zhou et al., "Objects as Points" (2019)
- GitHub：`xingyizhou/CenterNet`

#### 2.4.4 CornerNet（2018）

**学什么**：

- 核心思想：检测目标边界框的左上角和右下角关键点，然后进行配对
- 角点池化（Corner Pooling）：帮助网络更好地定位角点
- Embedding向量：用于将同一目标的左上角和右下角配对

**学到什么程度**：了解其基本思想即可。

**学习时间**：2小时

**推荐资源**：

- 原论文：Law & Deng, "CornerNet: Detecting Objects as Paired Keypoints" (2018)

---

### 2.5 Transformer检测器

#### 2.5.1 DETR（Detection Transformer, 2020）

**学什么**：

- **核心创新**：
  - 将检测建模为集合预测问题（set prediction），直接输出N个预测框（N通常100）
  - Transformer Encoder-Decoder结构：
    - CNN提特征 -> 展平为序列 -> Transformer Encoder（全局自注意力） -> Transformer Decoder（用N个object query查询） -> FFN预测类和框
  - 二分图匹配损失（Bipartite Matching Loss）：用匈牙利算法将预测框和真实框一一匹配
  - 无需NMS、无需anchor、无需手工设计的正负样本分配
- **优势**：全局注意力机制天然适合大目标、稀疏场景
- **劣势**：训练收敛慢（需要500个epoch）、小目标检测差

**为什么学**：

DETR是将Transformer引入目标检测的开山之作，开创了DEtection TRansformer的范式。在自动驾驶中，DETR系列在BEV感知中有重要应用。

**学到什么程度**：

- 能画出DETR的完整架构图（CNN backbone + Transformer encoder + decoder + FFN heads）
- 理解object query的含义和作用
- 理解匈牙利匹配的原理
- 能用 `torchvision.models.detection` 或 Facebook DETR官方代码进行推理

**学习时间**：2-3天

**推荐资源**：

- 原论文：Carion et al., "End-to-End Object Detection with Transformers" (2020)
- 李沐论文精读："DETR论文精读"（B站）
- GitHub：`facebookresearch/detr`
- 知乎文章：搜索"DETR 详解"

#### 2.5.2 Deformable DETR（2021）

**学什么**：

- 解决DETR的两个问题：训练收敛慢、小目标检测差
- **核心创新 —— Deformable Attention**：
  - 不像标准Attention关注所有位置，只关注少量可学习的关键点（reference points周围）
  - 计算量从 $O(HW \times HW)$ 降到 $O(HW \times K)$，K是采样点数（通常4）
  - 允许在多尺度特征图上进行注意力
- 多尺度可变形注意力（Multi-Scale Deformable Attention）
- 训练epoch从500减少到50

**为什么学**：

Deformable DETR大幅提升了DETR的实用性和效率，是目前DETR系列的常用变体。

**学到什么程度**：

- 理解Deformable Attention的基本思想（稀疏注意力 vs 全局注意力）
- 知道reference points和offsets的概念
- 能用 `PaddleDetection` 或 GitHub实现进行推理

**学习时间**：1-2天

**推荐资源**：

- 原论文：Zhu et al., "Deformable DETR: Deformable Transformers for End-to-End Object Detection" (2021)
- GitHub：`fundamentalvision/Deformable-DETR`

#### 2.5.3 RT-DETR（2023）

**学什么**：

- **核心创新**：DETR系列的实时版本，首次达到与YOLO可比的推理速度
- 使用混合编码器（Hybrid Encoder）：高效融合多尺度特征
- 不确定性最小化查询选择（Uncertainty-minimal Query Selection）：提升Decoder的查询质量
- 可以通过调节backbone和encoder大小灵活控制模型规模
- 推理速度：RT-DETR-R50在T4 GPU上达到约114 FPS

**为什么学**：

RT-DETR证明了DETR架构也可以达到实时性，在自动驾驶的实时感知中具有应用潜力。

**学到什么程度**：

- 了解RT-DETR相比原始DETR的改进
- 能用Ultralytics的实现进行训练和推理
- 理解YOLO vs DETR在实时检测中的取舍

**学习时间**：1天

**推荐资源**：

- 原论文：Zhao et al., "DETRs Beat YOLOs on Real-time Object Detection" (2023)
- GitHub：`lyuwenyu/RT-DETR`，以及Ultralytics中已集成
- 知乎文章：搜索"RT-DETR 详解"

---

### 2.6 数据增强策略

**学什么**：

**Mosaic增强（YOLOv4提出）**：
- 将4张训练图像随机拼接成一张大图
- 优点：增加每个batch中的目标数量和多样性、丰富背景
- 对小目标检测特别有帮助
- YOLOv5/v8中默认开启

**MixUp增强**：
- 将两张图像按比例混合：$new\_image = \lambda \cdot image_1 + (1-\lambda) \cdot image_2$
- 标签也按比例混合
- 提升模型对遮挡和噪声的鲁棒性

**CutOut / Random Erasing**：
- 随机在图像上遮挡一个矩形区域
- 强迫模型不依赖某个局部特征

**Copy-Paste增强（2021）**：
- 将目标实例从一张图复制粘贴到另一张图
- 增加目标实例数量和上下文多样性
- 适用于实例分割和检测

**其他常用增强**：
- 随机翻转（水平/垂直）
- 随机缩放和裁剪
- 颜色抖动（亮度、对比度、饱和度、色调）
- 随机旋转
- 几何变换（仿射、透视）

**为什么学**：

数据增强是提升检测性能最直接有效的手段。自动驾驶场景中数据分布复杂（光照变化、天气变化、遮挡），好的增强策略可以显著提升模型泛化能力。

**学到什么程度**：

- 能用 `albumentations` 实现完整的数据增强流水线
- 能在YOLO的配置文件中调整Mosaic、MixUp等增强的参数
- 知道什么时候该关闭某些增强（如验证时关闭Mosaic）
- 能可视化增强后的图像和标注框

**学习时间**：1-2天

**推荐资源**：

- `albumentations` 库：https://albumentations.ai/
- YOLOv5/v8的数据增强源码（`ultralytics` 库中的 `augment.py`）
- GitHub：`hysts/pytorch-cutout`、`rwightman/pytorch-image-models`（timm中的augment模块）

---

### 2.7 实战：用Ultralytics YOLOv8/v10训练自定义数据集

**学什么**：

- 完整的检测项目流程：
  1. **数据准备**：
     - 标注工具使用（LabelImg、CVAT、Roboflow）
     - 数据集格式：YOLO格式（txt标注文件）或COCO格式（json标注文件）
     - 数据集组织：train/val/test 分割，目录结构
     - 编写 `data.yaml` 配置文件
  2. **模型训练**：
     - 选择模型规模（n/s/m/l/x）
     - 设置训练超参数（epoch、batch_size、imgsz、lr0等）
     - 使用预训练权重进行微调
     - 监控训练过程（loss曲线、mAP曲线）
  3. **模型评估**：
     - 在验证集上计算mAP
     - 分析各类别的AP
     - 混淆矩阵分析
  4. **模型推理**：
     - 单张图像推理
     - 视频推理
     - 批量推理
  5. **模型导出**：
     - 导出为ONNX格式
     - 导出为TensorRT格式（自动驾驶部署常用）
  6. **性能优化**：
     - 调整置信度阈值和NMS阈值
     - 测试时增强（TTA）
     - 模型剪枝和量化

**为什么学**：

这是将所有理论知识转化为实际能力的关键步骤。能训练、评估、部署检测模型是自动驾驶感知工程师的核心技能。

**学到什么程度**：

- 能独立完成一个完整的检测项目（从数据收集到模型部署）
- 能处理常见的工程问题（类别不平衡、小目标、训练不收敛等）
- 能导出ONNX模型并用推理引擎部署

**学习时间**：1-2周

**推荐资源**：

- Ultralytics官方文档和教程：https://docs.ultralytics.com/
- GitHub：`ultralytics/ultralytics`
- B站推荐视频：
  - "YOLOv8从零开始"系列
  - "YOLOv8训练自定义数据集完整教程"
- 标注工具：
  - LabelImg：https://github.com/HumanSignal/labelImg
  - CVAT：https://github.com/opencv/cvat（功能更强大，支持团队协作）
  - Roboflow：https://roflowlow.com/（在线标注+数据管理+增强，有免费额度）
- 数据集下载：
  - COCO：https://cocodataset.org/
  - KITTI（自动驾驶）：https://www.cvlibs.net/datasets/kitti/

**检验标准**：

- 能用YOLOv8在自定义数据集上训练到合理的mAP
- 能导出ONNX模型并用ONNX Runtime或TensorRT进行推理
- 能分析训练结果并诊断常见问题

---

## Part 3: 语义分割与实例分割

### 3.1 语义分割

#### 3.1.1 FCN（Fully Convolutional Networks, 2015）

**学什么**：

- 核心创新：将全连接层替换为1x1卷积，使网络可以接受任意大小的输入
- 上采样方法：转置卷积（Transposed Convolution，也叫反卷积Deconvolution）
- 跳跃连接（Skip Connection）：
  - FCN-32s：只用最深层特征上采样32倍
  - FCN-16s：融合pool5和pool4的特征
  - FCN-8s：进一步融合pool3的特征
  - 浅层特征提供细节（边缘），深层特征提供语义信息
- 逐像素分类：对每个像素预测类别

**为什么学**：

FCN是语义分割的开山之作，其"用卷积替代全连接"+"上采样"+"跳跃连接"的范式是后续所有分割网络的基础。

**学到什么程度**：

- 能画出FCN的网络结构图
- 理解转置卷积的工作原理（不是卷积的逆操作，而是带填充的卷积）
- 理解跳跃连接如何融合多尺度特征

**学习时间**：1天

**推荐资源**：

- 原论文：Long et al., "Fully Convolutional Networks for Semantic Segmentation" (2015)
- 《动手学深度学习》13.9节 "语义分割和数据集"
- GitHub：`shelhamer/fcn.berkeleyvision.org`

#### 3.1.2 U-Net（2015）

**学什么**：

- 编码器-解码器结构（Encoder-Decoder）：
  - 编码器（收缩路径）：逐步下采样提取特征
  - 解码器（扩展路径）：逐步上采样恢复分辨率
  - 跳跃连接：将编码器的特征拼接到对应的解码器层
- 每次上采样后与编码器特征concat，然后接两个3x3卷积
- 最初为医学图像分割设计，但广泛应用于各种分割任务

**为什么学**：

U-Net的编码器-解码器+跳跃连接结构是分割网络的基本范式，自动驾驶中的很多分割网络（如车道线检测）都基于U-Net。

**学到什么程度**：

- 能用PyTorch从零实现U-Net
- 理解编码器、解码器、跳跃连接的维度匹配问题

**学习时间**：1天

**推荐资源**：

- 原论文：Ronneberger et al., "U-Net: Convolutional Networks for Biomedical Image Segmentation" (2015)
- GitHub：`milesial/Pytorch-UNet`（经典PyTorch实现，star很多）
- 知乎文章：搜索"U-Net 详解"

#### 3.1.3 DeepLab系列

**学什么**：

**DeepLab v1（2015）**：
- 空洞卷积（Dilated/Atrous Convolution）：在不增加参数的情况下增大感受野
  - 标准卷积kernel=3覆盖3x3区域，rate=2的空洞卷积覆盖5x5区域
- 条件随机场（CRF）后处理：精化分割边界

**DeepLab v2（2017）**：
- ASPP（Atrous Spatial Pyramid Pooling）：
  - 并行使用多个不同rate的空洞卷积，捕获多尺度信息
  - 类似Inception的思想但在空洞卷积上实现

**DeepLab v3（2017）**：
- 改进ASPP：加入全局平均池化（捕获全局上下文）
- 去掉CRF后处理
- Multi-grid策略

**DeepLab v3+（2018）**：
- 加入解码器模块：在ASPP后加一个简单的解码器恢复边界细节
- 使用深度可分离卷积减少计算量
- backbone：Xception

**为什么学**：

DeepLab系列是语义分割的经典方法，空洞卷积和ASPP被广泛使用。在自动驾驶中，DeepLab v3+常用于场景分割、可行驶区域分割。

**学到什么程度**：

- 能解释空洞卷积的原理和优势
- 能画出ASPP模块的结构
- 能用 `torchvision.models.segmentation.deeplabv3_resnet50` 进行推理和微调

**学习时间**：2天

**推荐资源**：

- 原论文：Chen et al., "Rethinking Atrous Convolution for Semantic Image Segmentation" (DeepLab v3, 2017)
- Chen et al., "Encoder-Decoder with Atrous Separable Convolution for Semantic Image Segmentation" (DeepLab v3+, 2018)
- GitHub：`pytorch/vision` 中的DeepLab实现
- 知乎文章：搜索"DeepLab 系列 详解"

#### 3.1.4 SegFormer（2021）

**学什么**：

- **核心创新**：基于Vision Transformer的语义分割
- Mix Transformer（MiT）编码器：
  - 层级结构（类似CNN的多尺度），产生1/4、1/8、1/16、1/32的特征图
  - Overlap Patch Embedding：用重叠的patch划分替代不重叠划分，减少块效应
  - Efficient Self-Attention：降低计算复杂度
- 轻量级MLP解码器：
  - 不需要复杂的解码器，只用MLP融合多尺度特征
  - 所有层的特征先上采样到1/4分辨率再concat，然后过MLP
- 无位置编码（实验证明不需要也能很好工作）

**为什么学**：

SegFormer代表了Transformer在分割领域的应用，在自动驾驶的场景解析中表现出色。其轻量级解码器设计对实时部署很友好。

**学到什么程度**：

- 理解MiT编码器的设计思想
- 理解为什么SegFormer的MLP解码器如此简洁却有效
- 能用 `mmsegmentation` 库进行推理和训练

**学习时间**：1天

**推荐资源**：

- 原论文：Xie et al., "SegFormer: Simple and Efficient Design for Semantic Segmentation with Transformers" (2021)
- GitHub：`NVlabs/SegFormer`
- `mmsegmentation` 库：https://github.com/open-mmlab/mmsegmentation（统一的分割工具箱）

---

### 3.2 实例分割

#### 3.2.1 Mask R-CNN（2017）

**学什么**：

- 核心思想：在Faster R-CNN的基础上增加一个并行的mask预测分支
- 架构：Faster R-CNN（检测框 + 类别） + Mask Head（逐像素二值分割）
- RoI Align：替代RoI Pooling，消除量化误差，对分割精度至关重要
  - RoI Pooling有两次量化（坐标取整），导致特征和原图对不上
  - RoI Align用双线性插值，保留浮点坐标
- Mask Head：小的FCN，对每个RoI预测一个二值mask（逐类别独立预测）
- 训练时：用检测到的框内的GT mask计算loss；推理时：用预测的框裁剪mask

**为什么学**：

Mask R-CNN是实例分割的里程碑，在自动驾驶中用于精确分割每个物体实例（如每辆车的轮廓）。理解它也有助于理解后续的全景分割方法。

**学到什么程度**：

- 能画出Mask R-CNN的完整架构图
- 能解释RoI Align的工作原理及其与RoI Pooling的区别
- 能用 `torchvision.models.detection.maskrcnn_resnet50_fpn` 进行推理和微调

**学习时间**：1-2天

**推荐资源**：

- 原论文：He et al., "Mask R-CNN" (2017)
- 李沐论文精读（B站搜索"李沐 Mask R-CNN"）
- GitHub：`pytorch/vision` 中的Mask R-CNN实现
- Facebook Detectron2：https://github.com/facebookresearch/detectron2（最完整的实现）

#### 3.2.2 SOLO / SOLOv2（2020）

**学什么**：

- 核心思想：将实例分割建模为实例感知的分类问题
- SOLO：
  - 将图像划分为 S×S 网格
  - 如果目标中心落入某个网格，该网格负责预测该目标的mask
  - 每个网格预测C个二值mask（对应C个类别）
  - 不需要检测框，直接预测mask
- SOLOv2：引入动态卷积，mask kernel在特征图上动态生成

**学到什么程度**：了解基本思想。

**学习时间**：2小时

#### 3.2.3 YOLACT（2019）

**学什么**：

- 核心思想：实时实例分割
- 生成一组"原型mask"（prototype masks），然后为每个实例预测一组线性组合系数
- 最终mask = 原型mask的线性组合 + 裁剪
- 速度优势：可以在接近单阶段检测器的速度下实现实例分割

**为什么学**：

YOLACT证明了实时实例分割是可行的，在自动驾驶的实时感知中有应用潜力。

**学到什么程度**：了解其"原型mask + 系数组合"的核心思想。

**学习时间**：2小时

**推荐资源**：

- 原论文：Bolya et al., "YOLACT: Real-time Instance Segmentation" (2019)
- GitHub：`dbolya/yolact`

---

### 3.3 全景分割

#### 3.3.1 Panoptic FPN（2019）

**学什么**：

- 全景分割（Panoptic Segmentation）：同时完成语义分割（stuff类，如天空、道路）和实例分割（thing类，如车辆、行人）
- Panoptic Quality (PQ)：$PQ = \frac{\sum_{(p,g) \in TP} IoU(p,g)}{|TP| + \frac{1}{2}|FP| + \frac{1}{2}|FN|}$
- FPN + 语义分割分支：在FPN基础上加一个语义分割头
- 后处理：将实例分割和语义分割结果通过启发式规则融合

**学到什么程度**：了解全景分割的任务定义和PQ指标。

**学习时间**：2小时

#### 3.3.2 Mask2Former（2022）

**学什么**：

- 核心创新：统一架构，一个模型同时处理语义分割、实例分割和全景分割
- Masked Cross-Attention：在Transformer Decoder中，每个query只在预测mask内的区域做注意力
- 多尺度高分辨率特征处理
- 性能全面超越之前的专用方法

**学到什么程度**：

- 了解Mask2Former的统一设计思想
- 能用 `transformers` 库中的Mask2Former实现进行推理

**学习时间**：半天

**推荐资源**：

- 原论文：Cheng et al., "Masked-attention Mask Transformer for Universal Image Segmentation" (2022)
- GitHub：`facebookresearch/Mask2Former`
- HuggingFace Transformers中的实现

---

### 3.4 自动驾驶中的分割应用

#### 3.4.1 可行驶区域分割

**学什么**：

- 任务：将图像中的像素分为"可行驶"和"不可行驶"两类
- 常用方法：DeepLab v3+、BiSeNet（实时语义分割网络）
- 数据集：Cityscapes、BDD100K、ApolloScape
- 工程考虑：实时性要求（需要轻量级模型）

#### 3.4.2 车道线检测

**学什么**：

- 传统语义分割方法：逐像素分类（如SCNN）
- 关键点检测方法：检测车道线的关键点然后连接（将在Part 5详细讨论）
- 实例级别检测：每条车道线作为一个实例
- 数据集：CULane、TuSimple、LLAMAS

**学到什么程度**：

- 了解自动驾驶中分割任务的不同定义和方法
- 能用预训练的DeepLab/BiSeNet在驾驶场景图像上进行推理

**学习时间**：1天

**推荐资源**：

- Cityscapes数据集：https://www.cityscapes-dataset.com/
- BDD100K：https://www.bdd100k.com/
- GitHub：`PaddlePaddle/PaddleSeg`（百度的分割工具箱，包含很多预训练模型）
- `mmsegmentation`：https://github.com/open-mmlab/mmsegmentation

---

## Part 4: 目标跟踪

### 4.1 单目标跟踪（SOT）

#### 4.1.1 SiamFC（2016）

**学什么**：

- 核心思想：将跟踪建模为模板匹配问题
- 模板分支：第一帧的目标区域通过CNN提取模板特征
- 搜索分支：后续帧的搜索区域通过同一个CNN提取特征
- 互相关操作（Cross-correlation）：模板特征在搜索区域上滑动，响应最高的位置即目标位置
- 端到端训练：在大规模视频数据集上训练

**为什么学**：

SiamFC开创了Siamese跟踪范式，后续的SiamRPN、SiamCAR等都基于这个框架。

**学到什么程度**：

- 理解Siamese网络的基本结构
- 理解互相关操作在跟踪中的作用
- 能用PyTorch实现基本的SiamFC

**学习时间**：1天

**推荐资源**：

- 原论文：Bertinetto et al., "Fully-Convolutional Siamese Networks for Object Tracking" (2016)
- GitHub：`harlencv/siamfc-pytorch`

#### 4.1.2 SiamRPN（2018）

**学什么**：

- 核心创新：在Siamese框架中引入RPN
- 模板分支生成RPN的卷积核（kernel），搜索分支的特征作为输入
- 同时预测分类分数和边框回归
- 不需要在线微调，速度极快（160 FPS）

**学到什么程度**：理解RPN如何集成到Siamese框架中。

**学习时间**：半天

#### 4.1.3 SiamCAR（2020）

**学什么**：

- anchor-free的Siamese跟踪器
- 分类和回归解耦
- 基于中心点的预测

**学到什么程度**：了解anchor-free在跟踪中的应用。

**学习时间**：2小时

---

### 4.2 多目标跟踪（MOT）

#### 4.2.1 SORT（2016）—— 卡尔曼滤波 + 匈牙利算法

**学什么**：

- 整体流程：检测 -> 预测 -> 匹配 -> 更新
- **卡尔曼滤波在跟踪中的应用**（结合你的控制理论基础）：
  - 状态向量：$\mathbf{x} = [x, y, s, r, \dot{x}, \dot{y}, \dot{s}]^T$（中心坐标、面积、长宽比、速度）
  - 状态转移模型（匀速运动模型）：$\mathbf{x}_{k+1} = \mathbf{F}\mathbf{x}_k + \mathbf{w}_k$
  - 观测模型：$\mathbf{z}_k = \mathbf{H}\mathbf{x}_k + \mathbf{v}_k$
  - 预测步：$\hat{\mathbf{x}}_{k|k-1} = \mathbf{F}\hat{\mathbf{x}}_{k-1|k-1}$，$\mathbf{P}_{k|k-1} = \mathbf{F}\mathbf{P}_{k-1|k-1}\mathbf{F}^T + \mathbf{Q}$
  - 更新步：卡尔曼增益 $\mathbf{K} = \mathbf{P}_{k|k-1}\mathbf{H}^T(\mathbf{H}\mathbf{P}_{k|k-1}\mathbf{H}^T + \mathbf{R})^{-1}$
  - 你学过的现代控制理论中的状态估计部分与这里完全对应！
- **匈牙利算法（Hungarian Algorithm）**：
  - 用于最优分配：将检测框和跟踪轨迹一一匹配
  - 代价矩阵：用检测框和卡尔曼滤波预测框之间的IoU矩阵（1-IoU作为距离）
  - 时间复杂度 $O(n^3)$
- SORT的局限：只用IoU匹配，没有外观特征，身份切换（ID Switch）严重

**为什么学**：

SORT是自动驾驶多目标跟踪的基础框架。卡尔曼滤波是你控制理论知识的直接应用，可以帮你快速理解跟踪中的状态估计部分。

**学到什么程度**：

- 能手推卡尔曼滤波在跟踪中的应用（能写出状态转移矩阵F和观测矩阵H）
- 能用 `scipy.optimize.linear_sum_assignment` 实现匈牙利匹配
- 能从零实现一个SORT跟踪器
- 理解SORT中每个模块的作用

**学习时间**：2-3天

**推荐资源**：

- 原论文：Bewley et al., "Simple Online and Realtime Tracking" (SORT, 2016)
- GitHub：`abewley/sort`
- 卡尔曼滤波复习：《现代控制理论》教材中状态观测器/卡尔曼滤波部分
- B站搜索"卡尔曼滤波 直观理解"——强烈建议先看可视化理解再看公式
- 匈牙利算法：知乎文章搜索"匈牙利算法 详解"

#### 4.2.2 DeepSORT（2017）

**学什么**：

- 核心改进：在SORT基础上加入**外观特征**（Re-ID特征）
- 匹配流程（级联匹配）：
  1. 先用外观特征（余弦距离）进行匹配
  2. 再用IoU进行匹配（处理新出现的目标和外观匹配失败的情况）
- Re-ID网络：一个独立的CNN，为每个检测框提取128维外观特征向量
- 匹配代价：$c_{i,j} = \lambda \cdot d_{appearance} + (1-\lambda) \cdot d_{IoU}$
- 深度级联匹配：优先匹配长时间未匹配的轨迹（给它们更高的匹配优先级）

**为什么学**：

DeepSORT是工程中使用最广泛的多目标跟踪算法之一。在自动驾驶中，外观特征帮助在遮挡后重新识别车辆/行人。

**学到什么程度**：

- 能理解DeepSORT的完整匹配流程
- 能用现有的DeepSORT代码在检测结果上进行跟踪
- 理解Re-ID特征如何减少ID Switch

**学习时间**：1-2天

**推荐资源**：

- 原论文：Wojke et al., "Simple Online and Realtime Tracking with a Deep Association Metric" (DeepSORT, 2017)
- GitHub：
  - `mikel-brostrom/Yolov5_DeepSort_Pytorch`（最常用的DeepSORT实现）
  - `ZQPei/deep_sort_pytorch`
  - `nwojke/deep_sort`（原作者实现）

#### 4.2.3 ByteTrack（2022）

**学什么**：

- **核心创新**：不仅用高置信度的检测结果，还利用低置信度的检测结果
- 动机：很多真实的目标被检测器赋予了低置信度分数（如被遮挡的目标），传统方法直接丢弃这些检测结果，导致丢失目标
- 匹配策略（两轮匹配）：
  1. 第一轮：将高置信度检测框与跟踪轨迹匹配（用卡尔曼滤波预测框的IoU）
  2. 第二轮：将低置信度检测框与剩余未匹配的轨迹匹配
- 效果：在多个MOT基准测试上达到SOTA

**为什么学**：

ByteTrack是目前最简单有效的跟踪算法之一，工程实用性极高。其思想——"低分检测也有价值"——很重要。

**学到什么程度**：

- 理解两轮匹配的策略及其动机
- 能用官方代码进行跟踪
- 理解为什么ByteTrack能在不使用Re-ID特征的情况下超越DeepSORT

**学习时间**：1天

**推荐资源**：

- 原论文：Zhang et al., "ByteTrack: Multi-Object Tracking by Associating Every Detection Box" (2022)
- GitHub：`ifzhang/ByteTrack`
- 李沐论文精读视频（B站搜索"李沐 ByteTrack"）

#### 4.2.4 BoT-SORT（2023）

**学什么**：

- 结合了DeepSORT的外观特征和ByteTrack的低分检测利用策略
- 加入了相机运动补偿（Global Motion Compensation, GMC）
- 改进了IoU计算（用GIoU替代IoU）
- 在MOT17/MOT20上达到当时的SOTA

**学到什么程度**：了解其核心改进。

**学习时间**：2小时

**推荐资源**：

- 原论文：Aharon et al., "BoT-SORT: Robust Associations Multi-Pedestrian Tracking" (2023)
- GitHub：`NirAharon/BoT-SORT`

---

### 4.3 3D多目标跟踪

#### 4.3.1 AB3DMOT（2020）

**学什么**：

- 将SORT框架扩展到3D：用3D卡尔曼滤波跟踪3D检测框
- 状态向量扩展到3D：$(x, y, z, l, w, h, \theta, \dot{x}, \dot{y}, \dot{z})$
- IoU计算在3D空间中进行（3D IoU或Bird's Eye View IoU）
- 简单但有效，是很多3D跟踪系统的baseline

**学到什么程度**：

- 理解3D跟踪与2D跟踪的区别
- 了解BEV IoU的计算方法

**学习时间**：半天

**推荐资源**：

- 原论文：Weng et al., "AB3DMOT: A Baseline for 3D Multi-Object Tracking and New Evaluation Metrics" (2020)
- GitHub：`xinshuoweng/AB3DMOT`

#### 4.3.2 CenterPoint-tracker（2021）

**学什么**：

- 基于CenterPoint 3D检测器的跟踪
- 跟踪策略：用检测到的目标中心点的速度估计进行预测匹配
- 不需要单独的跟踪模块，速度信息从检测器中直接获得
- 在Waymo和nuScenes上表现优异

**学到什么程度**：了解其"检测即跟踪"的思路。

**学习时间**：2小时

**推荐资源**：

- 原论文：Yin et al., "Center-based 3D Object Detection and Tracking" (2021)
- GitHub：`tianweiy/CenterPoint`

---

### 4.4 自动驾驶中的多传感器跟踪

**学什么**：

- 多传感器融合跟踪的基本概念：
  - 摄像头：提供2D检测框、类别、外观特征
  - 激光雷达：提供3D位置信息
  - 毫米波雷达：提供速度信息
- 融合策略：
  - 后融合（Late Fusion）：各传感器独立检测，然后在跟踪层面融合
  - 前融合（Early Fusion）：在原始数据层面融合
  - 特征级融合（Feature-level Fusion）：在特征层面融合
- 时间同步问题：不同传感器的时间戳对齐

**学到什么程度**：

- 了解多传感器融合的基本概念和策略
- 不需要深入实现，但要理解在跟踪框架中如何融合不同来源的检测结果

**学习时间**：半天

**推荐资源**：

- 知乎文章：搜索"自动驾驶 多传感器融合 跟踪"
- 论文：搜索 "multi-sensor fusion tracking autonomous driving"

**多目标跟踪整体检验标准**：

- 能用DeepSORT或ByteTrack在检测结果上实现多目标跟踪
- 能解释卡尔曼滤波在跟踪中的完整工作流程（能关联到控制理论中的状态估计）
- 能解释SORT/DeepSORT/ByteTrack各自的优缺点

---

## Part 5: 关键点检测与姿态估计

### 5.1 2D关键点检测

#### 5.1.1 HRNet（High-Resolution Network, 2019）

**学什么**：

- 核心创新：始终保持高分辨率特征表示（不像之前的网络先下采样再上采样）
- 多分辨率并行：同时维护4个分辨率的特征流（1/4, 1/8, 1/16, 1/32）
- 多尺度融合：不同分辨率之间的特征反复交换信息
- 输出：每个关键点的热力图（Heatmap），取热力图最大值位置作为关键点坐标
- 关键点回归方法：直接回归坐标（不如热力图精确）vs 热力图回归

**为什么学**：

HRNet是关键点检测的SOTA方法之一，在自动驾驶中用于行人姿态估计、驾驶员行为分析等。其多分辨率并行的设计思想也被用于其他任务。

**学到什么程度**：

- 能理解HRNet的多分辨率并行架构
- 能用热力图方法训练关键点检测模型
- 能用 `mmpose` 库进行关键点检测推理

**学习时间**：1-2天

**推荐资源**：

- 原论文：Sun et al., "Deep High-Resolution Representation Learning for Visual Recognition" (2019)
- GitHub：`leoxiaobin/deep-high-resolution-net.pytorch`（原作者PyTorch实现）
- `mmpose` 库：https://github.com/open-mmlab/mmpose（最完整的姿态估计工具箱）

#### 5.1.2 SimpleBaseline（2018）

**学什么**：

- 架构非常简单：ResNet backbone + 几个反卷积层（上采样） -> 热力图输出
- 证明了"简单的架构 + 足够的数据"也能达到很好的效果
- 作为关键点检测的baseline方法

**学到什么程度**：

- 能实现一个简单的关键点检测模型
- 理解热力图回归的损失函数（MSE Loss，对预测热力图和GT高斯热力图计算）

**学习时间**：半天

**推荐资源**：

- 原论文：Xiao et al., "Simple Baselines for Human Pose Estimation and Tracking" (2018)
- GitHub：`Microsoft/human-pose-estimation.pytorch`

---

### 5.2 车道线检测

#### 5.2.1 LaneNet（2018）

**学什么**：

- 实例级车道线检测：将车道线检测分为语义分割 + 实例嵌入
- 语义分割分支：逐像素分类（车道线 or 背景）
- 实例嵌入分支：为每个车道线像素学习一个嵌入向量，同一车道线的像素嵌入向量相近，不同车道线的嵌入向量远离
- 后处理：用聚类算法（如MeanShift）将嵌入向量聚类，区分不同的车道线
- H-Nets：用透视变换拟合车道线

**为什么学**：

LaneNet是实例级车道线检测的经典方法，理解其"分割+嵌入"的思路对理解后续方法有帮助。

**学到什么程度**：

- 理解实例嵌入的损失函数（pull loss + push loss）
- 能用现有代码进行车道线检测

**学习时间**：半天

**推荐资源**：

- 原论文：Neven et al., "Towards End-to-End Lane Detection: an Instance Segmentation Approach" (2018)
- GitHub：`MaybeShewill-CV/lanenet-lane-detection`（TensorFlow实现）

#### 5.2.2 CLRNet（2022）

**学什么**：

- 核心思想：从粗到细的车道线检测
- Cross Layer Refinement：利用不同层的特征逐步精化车道线预测
- Lane IoU Loss：专门设计的车道线IoU损失
- 基于anchor的车道线表示：用预定义的anchor线作为初始预测

**为什么学**：

CLRNet是当前车道线检测的SOTA方法之一，在CULane基准上表现优异。

**学到什么程度**：

- 理解从粗到细的检测策略
- 能用官方代码在CULane上评估

**学习时间**：1天

**推荐资源**：

- 原论文：Zheng et al., "CLRNet: Cross Layer Refinement Network for Lane Detection" (2022)
- GitHub：`Turoad/CLRNet`

#### 5.2.3 CLRerNet（2023）

**学什么**：

- CLRNet的改进版本
- 引入了更强的NMS策略和分类头设计
- 在多个基准上刷新SOTA

**学到什么程度**：了解其相比CLRNet的改进。

**学习时间**：2小时

**推荐资源**：

- GitHub：`hirokicamera/CLRerNet`

---

### 5.3 自动驾驶中的应用

#### 5.3.1 行人姿态估计

**学什么**：

- 任务：检测图像中每个行人的关键点（通常17个关键点，COCO格式）
- 应用：
  - 行人意图预测（通过姿态判断行人是否准备过马路）
  - 自动驾驶安全区域判断
- 流程：先用检测器检测行人 -> 对每个行人裁剪区域进行关键点检测
- 代表方法：HRNet + top-down（先检测再关键点检测）

**学到什么程度**：

- 能用 `mmpose` 或 Ultralytics的pose模型对驾驶场景图像进行姿态估计
- 了解top-down和bottom-up两种范式的区别

**学习时间**：半天

**推荐资源**：

- `mmpose`：https://github.com/open-mmlab/mmpose
- Ultralytics YOLOv8-pose：在Ultralytics框架中直接支持

#### 5.3.2 车辆朝向估计

**学什么**：

- 任务：估计车辆的朝向角（heading angle），对3D定位很重要
- 方法：通过检测车辆的关键点（如8个角点）来估计朝向
- 在单目3D检测中，车辆朝向是关键的中间表示

**学到什么程度**：了解关键点如何用于车辆朝向估计。

**学习时间**：2小时

---

## 整体学习路径建议（时间规划）

按以下顺序学习，预计总时间 4-6 个月（每天学习 6-8 小时）：

**第一阶段（1-1.5个月）：深度学习基础 + PyTorch**
- Week 1-2：神经网络基本原理、反向传播推导、NumPy实现
- Week 3-4：CNN架构演进、PyTorch基本操作
- Week 5-6：损失函数、优化器、正则化、训练技巧、迁移学习

**第二阶段（1-1.5个月）：目标检测**
- Week 7-8：检测基础概念、Faster R-CNN
- Week 9-10：YOLO系列（重点v3、v5、v8）、实战训练
- Week 11-12：Anchor-free方法、DETR系列、数据增强

**第三阶段（1个月）：分割**
- Week 13-14：FCN、U-Net、DeepLab系列
- Week 15-16：实例分割（Mask R-CNN）、全景分割、SegFormer

**第四阶段（0.5-1个月）：跟踪**
- Week 17-18：SORT/DeepSORT/ByteTrack
- Week 19：3D跟踪、多传感器跟踪

**第五阶段（0.5个月）：关键点检测 + 车道线**
- Week 20：HRNet、车道线检测

**贯穿始终的实践**：
- 每学完一个模块，都要跑通代码并训练模型
- 积累至少一个完整的项目经验（如"基于YOLOv8的自动驾驶目标检测"）
- 记录学习笔记和实验日志

---

## 补充：核心工具链

以下工具是贯穿整个学习过程必须掌握的：

| 工具 | 用途 | 学习资源 |
|------|------|---------|
| **PyTorch** | 深度学习框架 | 官方教程 + 《动手学深度学习》 |
| **OpenCV** | 图像处理 | `cv2` 库，B站搜索"OpenCV Python" |
| **NumPy** | 数值计算 | 任何Python数据科学教程 |
| **Matplotlib** | 可视化 | 绑定OpenCV和NumPy学习 |
| **Ultralytics** | YOLO框架 | 官方文档 |
| **timm** | 预训练模型库 | GitHub：`huggingface/pytorch-image-models` |
| **mmcv/mmdet/mmseg/mmpose** | OpenMMLab系列工具箱 | 各自的GitHub和文档 |
| **Weights & Biases (Wandb)** | 实验管理 | https://wandb.ai/ |
| **LabelImg / CVAT** | 数据标注 | GitHub |
| **ONNX Runtime / TensorRT** | 模型推理部署 | 官方文档 |

---

以上就是完整的自动驾驶 2D 感知学习指南。每个技术点都标注了"学什么、为什么学、学到什么程度"，并附有具体的学习资源。建议在学习过程中保持"理论 + 代码实践"的节奏，每学完一个模块就跑通对应代码并训练一个模型，这样效果最好。

---

# 模块三：SLAM与定位技术

## 自动驾驶方向 - SLAM与定位技术 学习指南

> 面向对象：机器人工程本科背景，已掌握自动控制原理与现代控制理论（状态空间、卡尔曼滤波、LQR），即将进入自动驾驶定位方向读研。

---

## Part 1: 视觉SLAM

### 1.1 SLAM问题定义（前端/后端/回环检测/建图）

**学什么**：理解SLAM（Simultaneous Localization and Mapping）的核心问题是"从未知环境的未知位置出发，在运动过程中通过重复观测到的环境特征定位自身位置和姿态，再根据自身位置增量式地构建周围环境地图"。掌握前端视觉里程计（帧间位姿估计）、后端非线性优化（全局一致性优化）、回环检测（消除累计漂移）、建图（稀疏/稠密/语义地图）四大模块的功能与数据流。

**为什么学**：这是整个SLAM系统的全局架构认知。不理解模块分工，后续学习每个子模块时会只见树木不见森林。在工业落地中，系统集成能力往往比单一算法更重要。

**学到什么程度**：能画出完整的SLAM系统流程图，说明每个模块的输入输出、数据类型、运行频率、失败时的降级策略。能对比基于滤波和基于优化的两种框架的优劣。

**学习时间**：1周

**推荐资源**：
- 书籍：高翔《视觉SLAM十四讲：从理论到实践》第1-3章（必读，入门经典）
- 书籍：Cyrill Stachniss《SLAM for Dummies》（入门速读）
- 课程：高翔B站配套视频 https://www.bilibili.com/video/BV16t411Y7mQ
- 课程：TUM Cremers教授Visual SLAM公开课 https://www.youtube.com/playlist?list=PLTBdjVg4VjxofGDfT7Ci2c86bAcXFrgMa
- 博客：泡泡机器人SLAM微信公众号及博客 http://www.paopaorobot.org

**检验标准**：能口述SLAM四大模块及数据流；能在白板上画出系统架构图。

---

### 1.2 视觉里程计VO - 特征点法

**学什么**：FAST角点检测（快速但无方向性）、BRIEF二进制描述子（高效匹配）、ORB特征（Oriented FAST + Rotated BRIEF，解决了旋转不变性和速度问题）。特征匹配策略：暴力匹配、FLANN快速匹配、比率测试（Lowe's ratio test）筛选误匹配。RANSAC/PROSAC剔除外点。

**为什么学**：特征点法是视觉SLAM最经典、最成熟的前端方案，ORB-SLAM3即以此为核心。理解特征提取-匹配-估计的完整pipeline是做视觉定位的基本功。在自动驾驶中，特征点法用于视觉定位退化检测、与LiDAR配准的初始化等场景。

**学到什么程度**：能手写ORB特征提取流程（金字塔构建、FAST检测、BRIEF计算）；能解释RANSAC的原理和采样次数计算；能用OpenCV实现完整的特征提取匹配pipeline。

**学习时间**：2周

**推荐资源**：
- 书籍：《视觉SLAM十四讲》第7章
- 论文：ORB-SLAM论文 Rublee et al., "ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras"
- 课程：深蓝学院《视觉SLAM进阶》（陈建讲师，含工程实践）
- GitHub：OpenCV特征匹配教程 https://github.com/opencv/opencv
- GitHub：ORB-SLAM3 https://github.com/UZ-SLAMLab/ORB_SLAM3
- B站：ORB特征讲解 https://www.bilibili.com/video/BV16t411Y7mQ 对应章节

**检验标准**：能在OpenCV中实现ORB特征提取+匹配+RANSAC估计基本矩阵，输出匹配可视化图。

---

### 1.3 视觉里程计VO - 直接法与光流法

**学什么**：Lucas-Kanade稀疏光流（假设局部光度不变、小运动、用高斯牛顿求解像素偏移）、LK光流的金字塔实现（应对大运动）。直接法VO（DSO为代表）：最小化光度误差而非几何重投影误差，建立photometric error的雅可比，直接优化相机位姿。半稠密直接法（利用梯度显著像素）vs 稠密直接法。

**为什么学**：直接法不依赖特征描述子，在纹理贫乏区域也能工作，且输出更稠密。DSO是直接法代表作。理解直接法有助于理解自动驾驶中光度标定、曝光补偿等工程问题。

**学到什么程度**：能推导LK光流的数学过程（图像亮度不变约束 -> 雅可比矩阵 -> 最小二乘求解）；能解释直接法BA与特征点法BA的目标函数区别；能跑通DSO demo。

**学习时间**：1.5周

**推荐资源**：
- 书籍：《视觉SLAM十四讲》第8章
- 论文：Engel et al., "LSD-SLAM" 和 "DSO"
- 论文：Baker & Matthews, "Lucas-Kanade 20 Years On"（LK光流经典综述）
- GitHub：DSO https://github.com/JakobEngel/dso
- GitHub：光流法OpenCV示例 https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html

**检验标准**：能用OpenCV实现LK光流追踪并可视化；能说出直接法与特征点法在目标函数、适用场景、计算量上的三方面差异。

---

### 1.4 2D-2D对极几何

**学什么**：对极约束的几何含义（极平面、极线、极点），本质矩阵E（内参归一化坐标下）和基础矩阵F（像素坐标下）的定义、自由度、求解方法（八点法、五点法），单应矩阵H（适用于平面场景），如何从E/H恢复R,t（SVD分解），三角化恢复3D点。

**为什么学**：对极几何是单目SLAM初始化的核心，也是理解相机运动几何关系的基础。在自动驾驶中，纯视觉初始化（如VINS初始化阶段）依赖对极几何。

**学到什么程度**：能手推八点法SVD求解E的过程；能判断何时用E何时用H（平面场景检测，通过E和H的重投影误差比较）；能从E通过SVD得到4组R,t解并选出正确解。

**学习时间**：1周

**推荐资源**：
- 书籍：《视觉SLAM十四讲》第7.3-7.5节
- 书籍：Hartley & Zisserman《Multiple View Geometry in Computer Vision》第9-11章（经典参考）
- 课程：Cyrill Stachniss的Visual SLAM课程中对极几何部分
- 工具：《视觉SLAM十四讲》配套代码 https://github.com/gaoxiang12/slambook2

**检验标准**：能从匹配点对计算E矩阵，SVD分解得到R,t，三角化得到3D点并用OpenCV验证。

---

### 1.5 3D-2D PnP问题

**学什么**：PnP（Perspective-n-Point）问题定义——已知3D地图点和对应2D像素，求解相机位姿。P3P（最少3个点对，利用余弦定理，多解）、EPnP（将3D点表示为4个控制点的加权组合，线性求解，O(n)复杂度）、DLS/UPnP等。非线性优化解法：构建重投影误差，用Gauss-Newton/LM迭代求解。OpenCV solvePnP/solvePnPRansac的使用。

**为什么学**：PnP是视觉SLAM中最频繁使用的位姿估计方式——每一帧新图像都可以通过与已有地图点匹配用PnP求位姿。在自动驾驶视觉定位中，将视觉特征与高精地图匹配也依赖PnP。

**学到什么程度**：能推导EPnP的线性化过程；能对比P3P、EPnP、迭代法的适用场景（点数、实时性、精度）；能在代码中熟练使用cv::solvePnP。

**学习时间**：1.5周

**推荐资源**：
- 书籍：《视觉SLAM十四讲》第7.6节
- 论文：Lepetit et al., "EPnP: An Accurate O(n) Solution to the PnP Problem"
- 论文：Gao et al., "Complete Solution Classification for the P3P Problem"
- GitHub：OpenCV PnP文档 https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html

**检验标准**：能用3D-2D对应点对求解位姿，对比不同PnP方法的精度和耗时。

---

### 1.6 3D-3D ICP

**学什么**：ICP（Iterative Closest Point）问题定义——已知两组3D点云，求解刚体变换R,t。SVD闭式解法：计算质心、去质心、构建协方差矩阵、SVD分解得到R，再算t。非线性优化解法：构建点到点误差，用Gauss-Newton迭代。点到面ICP（point-to-plane）的法向量约束。

**为什么学**：ICP是LiDAR SLAM点云配准的基础，也是RGB-D SLAM的常用方法。理解SVD求解为后续学习SVD在SLAM中的其他应用（如SVD求ICP、BA中的Schur complement）打好基础。

**学到什么程度**：能手推SVD求解R,t的完整过程；能实现基础ICP（点到点）；了解point-to-plane ICP的优势。

**学习时间**：1周

**推荐资源**：
- 书籍：《视觉SLAM十四讲》第7.7节
- 论文：Besl & McKay, "A Method for Registration of 3D Shapes"（ICP原始论文）
- GitHub：Open3D ICP教程 http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html
- 博客：ICP算法详解 https://www.cnblogs.com/feifanrensheng/p/12388498.html

**检验标准**：能用SVD手写ICP求解，与Open3D的ICP结果对比验证。

---

### 1.7 后端优化（BA/图优化/滑动窗口/边缘化）

**学什么**：Bundle Adjustment（BA）定义——同时优化相机位姿和地图点，最小化重投影误差总和。雅可比矩阵推导（位姿对重投影误差的导数、地图点对重投影误差的导数）。Hessian矩阵的稀疏结构与Schur complement（先消去地图点，优化位姿）。图优化框架g2o（顶点/边的定义）、GTSAM（因子图）、Ceres Solver（自动求导）的使用。滑动窗口（维持计算量可控）与边缘化（将老帧信息压缩为先验约束）的原理与工程实现。

**为什么学**：后端优化决定了SLAM系统的精度上限。滑动窗口+边缘化是VINS-Fusion等主流系统的核心架构。在工业落地中，边缘化带来的fill-in问题、矩阵条件数恶化等是调参和工程优化的重点。

**学到什么程度**：能推导BA的雅可比矩阵和正规方程；能用Ceres/GTSAM实现一个简单的BA；理解Schur complement的物理意义；能解释边缘化如何保留老帧的信息同时保持问题规模可控。

**学习时间**：3周（重点投入）

**推荐资源**：
- 书籍：《视觉SLAM十四讲》第10-11章
- 书籍：Triggs et al., "Bundle Adjustment — A Modern Synthesis"（BA经典综述）
- 课程：深蓝学院《SLAM后端优化》专题
- GitHub：g2o https://github.com/RainerKuemmerle/g2o
- GitHub：GTSAM https://github.com/borglab/g2o（Georgia Tech出品，文档完善）
- GitHub：Ceres Solver http://ceres-solver.org
- GitHub：VINS-Fusion https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- 博客：从零开始手写BA https://zhuanlan.zhihu.com/p/430842639

**检验标准**：能用GTSAM/Ceres实现简单的BA优化；能解释Schur complement的步骤；能说出边缘化的具体实现步骤（哪些变量被边缘化、如何构造先验信息矩阵）。

---

### 1.8 回环检测

**学什么**：回环检测的意义（消除累计漂移）。词袋模型（Bag of Words）：用DBoW2/DBoW3将ORB特征聚类为视觉词典，将图像表示为词袋向量，通过向量相似度判断是否回环。NetVLAD（基于CNN的可训练全局描述子，端到端学习）。CosPlace（CVPR 2022，轻量级分组策略的地点识别）。回环验证（Sim(3)相对位姿计算、回环融合/位姿图优化）。

**为什么学**：没有回环检测，SLAM系统在长距离运行后必然漂移。在自动驾驶中，停车场等场景尤其依赖回环。NetVLAD/CosPlace等深度学习方法已在工业界广泛部署。

**学到什么程度**：能解释词袋模型的层次化聚类和倒排索引结构；能训练一个小型DBoW词典；了解NetVLAD的VLAD编码原理。

**学习时间**：1.5周

**推荐资源**：
- 书籍：《视觉SLAM十四讲》第12章
- 论文：Galvez-Lopez & Tardos, "Bags of Binary Words for Fast Place Recognition in Image Sequences"（DBoW2）
- 论文：Arandjelovic et al., "NetVLAD: CNN Architecture for Weakly Supervised Place Recognition"
- 论文：Berton et al., "Rethinking Visual Geo-localization for Large-Scale Applications"（CosPlace）
- GitHub：DBoW3 https://github.com/rmsalinas/DBow3
- GitHub：NetVLAD PyTorch实现 https://github.com/Relja/netvlad_tf_open

**检验标准**：能用DBoW3构建词典并在自己的数据集上检测回环。

---

### 1.9 代表性视觉SLAM系统

**学什么**：
- **ORB-SLAM3**：单目/双目/RGB-D/IMU四模式支持，Atlas多地图系统，最大贡献是多地图合并与IMU初始化。
- **VINS-Fusion**：港科大出品，VIO系统标杆，单目/双目+IMU，支持GPS融合，滑动窗口优化+回环检测。
- **LSD-SLAM**：半稠密直接法SLAM，维护半稠密深度图。
- **DSO**：直接法稀疏里程计，联合优化相机位姿、曝光时间和光度参数。

**为什么学**：工程落地必须在开源系统上二次开发。ORB-SLAM3和VINS-Fusion是目前工业界最常用的两个视觉SLAM框架，掌握它们的代码架构是做项目的基础。

**学到什么程度**：能读懂ORB-SLAM3/VINS-Fusion的主流程代码，知道数据如何从输入到输出；能在自己的数据集上跑通demo并做基本调参；了解每个系统的优势场景和已知缺陷。

**学习时间**：4周（含代码阅读与实验）

**推荐资源**：
- GitHub：ORB-SLAM3 https://github.com/UZ-SLAMLab/ORB_SLAM3
- GitHub：VINS-Fusion https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- GitHub：DSO https://github.com/JakobEngel/dso
- 数据集：EuRoC MAV Dataset https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
- 数据集：TUM RGB-D Dataset https://cvg.cit.tum.de/data/datasets/rgbd-dataset
- 教程：ORB-SLAM3代码详解（知乎系列搜索"ORB-SLAM3源码解读"）

**检验标准**：能在EuRoC数据集上跑通ORB-SLAM3（视觉+IMU模式）和VINS-Fusion，输出轨迹并与ground truth对比计算ATE/RPE。

---

## Part 2: LiDAR SLAM（自动驾驶主流）

### 2.1 LiDAR传感器原理

**学什么**：激光测距原理（飞行时间ToF / 相位法）。机械旋转式LiDAR（Velodyne VLP-16/128线，360度旋转扫描）、固态LiDAR（MEMS微振镜、转镜式、OPA光学相控阵）、Flash LiDAR（面阵一次成像）。点云格式（x,y,z,intensity,ring,timestamp）。关键参数：线数、点频、测距精度、角分辨率、FOV。

**为什么学**：传感器特性直接决定SLAM算法设计。例如固态LiDAR的非重复扫描特性导致点云稀疏，需要不同的处理策略。工程落地中LiDAR选型、安装标定都依赖对传感器的深入理解。

**学到什么程度**：能解释ToF测距原理；能区分不同LiDAR类型的特点和适用场景；能处理LiDAR原始数据（解析pcap包、转换点云格式）。

**学习时间**：0.5周

**推荐资源**：
- 书籍：Thrun et al., "Probabilistic Robotics" 第6章（传感器模型）
- 文档：Velodyne VLP-16用户手册（理解真实传感器参数）
- GitHub：velodyne驱动 https://github.com/ros-drivers/velodyne_driver
- 博客：LiDAR传感器原理科普 https://zhuanlan.zhihu.com/p/94425229

**检验标准**：能说出三种LiDAR的扫描原理差异；能解析一个Velodyne pcap文件并可视化点云。

---

### 2.2 点云处理基础

**学什么**：PCL（Point Cloud Library）和Open3D两大工具库的使用。点云滤波：体素降采样（Voxel Grid）、统计滤波（去除离群点）、半径滤波。点云配准：ICP（迭代最近点）、NDT（正态分布变换，将点云体素化为正态分布，用概率模型配准，对初始值更鲁棒）。点云分割：RANSAC地面去除、欧氏聚类（DBSCAN）、区域生长。

**为什么学**：点云处理是LiDAR SLAM的基础能力。NDT是很多LiDAR定位系统的核心配准算法。工程中大量时间花在点云预处理上。

**学到什么程度**：熟练使用PCL/Open3D进行滤波、配准、分割；理解NDT的数学原理（体素化 -> 用正态分布建模 -> 构建Hessian求解）；能处理百万级点云的实时性问题。

**学习时间**：2周

**推荐资源**：
- 书籍：PCL官方教程 http://pointclouds.org/documentation
- 文档：Open3D教程 http://www.open3d.org/docs/release/
- 课程：Stanford CS231A中3D Vision相关章节
- GitHub：Open3D https://github.com/isl-org/Open3D
- GitHub：PCL https://github.com/PointCloudLibrary/pcl
- 博客：NDT算法详解 https://zhuanlan.zhihu.com/p/361658112

**检验标准**：能用PCL对Kitti点云做地面去除+聚类+可视化；能用NDT完成两帧点云配准。

---

### 2.3 经典LiDAR SLAM系统

**学什么**：
- **LOAM**（Ji Zhang, 2014）：LiDAR里程计的开山之作，将点分为边缘点和平面点，分别构建点到线、点到面的匹配约束，低频做地图优化、高频做里程计。
- **LeGO-LOAM**：LOAM的轻量化改进，加入地面分割（利用地面平面约束减少自由度）、图优化后端。
- **LIO-SAM**：因子图框架（GTSAM），紧耦合LiDAR-IMU，支持GPS因子，是目前综合性能最好的LiDAR-惯性SLAM之一。
- **FAST-LIO2**：紧耦合EKF框架，ikd-Tree增量式地图维护，极致效率，可在嵌入式平台实时运行。
- **Point-LIO**：逐点处理（而非帧处理），极致低延迟，适用于高速运动。
- **KISS-ICP**："Keep It Small and Simple"，极简ICP里程计，无需IMU、无需调参，用体素化的点到点ICP实现。

**为什么学**：LOAM系列是自动驾驶LiDAR SLAM的事实标准，理解其特征提取-匹配-优化的框架是做LiDAR定位的基础。LIO-SAM和FAST-LIO2是当前工业界最常用的开源系统。KISS-ICP代表了"简洁有效"的工程哲学。

**学到什么程度**：能解释LOAM的边缘点/平面点特征提取原理和匹配残差的构建；能跑通FAST-LIO2/LIO-SAM的demo；了解各系统的适用场景（室内/室外、高动态/低动态、有无IMU）。

**学习时间**：4周（重点投入）

**推荐资源**：
- 论文：Zhang & Singh, "LOAM: Lidar Odometry and Mapping in Real-time"
- 论文：Shan & Englot, "LeGO-LOAM"
- 论文：Shan et al., "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping"
- 论文：Xu & Zhang, "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry"
- 论文：Cao et al., "Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry"
- 论文：Vizzo et al., "KISS-ICP: In Defense of Point-to-Point ICP"
- GitHub：FAST-LIO2 https://github.com/hku-mars/FAST_LIO
- GitHub：LIO-SAM https://github.com/TixiaoShan/LIO-SAM
- GitHub：KISS-ICP https://github.com/PRBonn/kiss-icp
- GitHub：LeGO-LOAM https://github.com/RobustFieldAutonomyLab/LeGO-LOAM
- 数据集：Kitti https://www.cvlibs.net/datasets/kitti/
- B站：FAST-LIO2讲解 https://www.bilibili.com/video/BV1HG4y1M7s4

**检验标准**：能在Kitti数据集上跑通LIO-SAM/FAST-LIO2，理解点云帧到地图的匹配过程；能对比不同系统在相同数据集上的精度和效率。

---

## Part 3: 惯性导航与组合导航

> 注：本学生已掌握卡尔曼滤波、状态空间模型、最优控制LQR，因此可以在已有控制理论基础上深化理解。

### 3.1 IMU传感器原理

**学什么**：加速度计（MEMS微机械结构，测量比力f=a-g，包含重力）、陀螺仪（科氏力效应，测量角速度）、磁力计（测量地磁北向，提供航向约束）。IMU误差模型：零偏（bias）、比例因子误差、噪声（白噪声+随机游走，用Allan方差标定）。IMU坐标系约定（FRD/NED）。

**为什么学**：IMU误差模型是IMU预积分和EKF融合的数学基础。工程中IMU标定（六面法、转台法）直接影响定位精度。

**学到什么程度**：能写出加速度计和陀螺仪的测量方程（含bias和噪声项）；能解释Allan方差曲线的含义并标定IMU参数；理解bias随机游走的物理意义。

**学习时间**：1周

**推荐资源**：
- 书籍：Titterton & Weston, "Strapdown Inertial Navigation Technology"（惯导经典教材）
- 论文：Allan方差标定方法 IEEE Standard 952
- 工具：imu_utils（基于ROS的IMU标定工具）https://github.com/gaowenliang/imu_utils
- 工具：Kalibr（相机-IMU联合标定）https://github.com/ethz-asl/kalibr
- 博客：IMU原理与Allan方差 https://zhuanlan.zhihu.com/p/341861505

**检验标准**：能用Allan方差工具标定一个IMU的噪声参数；能写出IMU测量方程。

---

### 3.2 IMU预积分（Preintegration）

**学什么**：IMU预积分的核心问题：在优化框架中，当相机位姿被优化更新后，两帧之间的IMU积分量需要重新积分——计算代价极大。预积分的做法是将IMU积分量定义为"与参考帧位姿无关"的增量量delta，优化更新参考帧后只需做一次简单的坐标旋转而非重新积分。预积分量的递推公式、协方差递推、bias的线性化修正（一阶近似）。

**为什么学**：IMU预积分是VIO（视觉惯性里程计）的核心数学工具，VINS-Fusion、ORB-SLAM3的IMU模块都基于此。理解预积分是理解紧耦合VIO的前提。你已经学过状态空间和卡尔曼滤波，可以把预积分理解为"连续时间状态方程的离散化积分"。

**学到什么程度**：能推导预积分量的递推公式和雅可比矩阵；能理解预积分误差在优化中的使用方式（作为IMU factor的残差项）；能在代码中找到预积分的实现。

**学习时间**：2周

**推荐资源**：
- 论文：Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"（预积分原始论文，必读）
- 论文：Qin et al., "VINS-Mono" 的附录（预积分推导清晰）
- 课程：深蓝学院《VIO入门与进阶》（贺博讲师，含推导）
- GitHub：VINS-Fusion https://github.com/HKUST-Aerial-Robotics/VINS-Fusion（预积分实现代码）
- 博客：IMU预积分详解 https://blog.csdn.net/qq_36172652/article/details/119809147

**检验标准**：能推导预积分增量的递推方程；能解释"为什么预积分量可以在不重新积分的情况下适用于新的参考位姿"。

---

### 3.3 卡尔曼滤波家族（结合已有控制理论基础）

**学什么**：你已经学过标准KF，此处需要深化：
- **EKF**（扩展卡尔曼滤波）：将非线性系统一阶线性化，predict-update循环。你已学过，此处重点是SLAM中的具体应用——EKF-SLAM的状态向量设计（位姿+所有地标点），理解其O(n^2)复杂度导致不可扩展的问题。
- **ESKF**（Error-State KF）：状态量为名义状态+小量误差状态，误差状态满足线性高斯假设，精度更高。FAST-LIO2、MSCKF均基于ESKF。这是与你学过的LQR/最优控制最直接相关的部分——误差状态动力学就是线性化的系统模型。
- **IEKF**（Iterated EKF）：在update阶段反复线性化（类似Gauss-Newton迭代），提升精度。LIO-SAM等系统使用。
- **UKF**（无迹卡尔曼滤波）：用sigma点捕捉非线性变换后的均值和方差，避免求雅可比。

**为什么学**：EKF/ESKF是滤波型LiDAR-Inertial融合的数学核心。理解ESKF有助于理解LIO系统的状态估计。你已有控制理论基础，可以把这些滤波器视为"最优状态估计器"的变体——predict对应状态方程传播，update对应测量修正。

**学到什么程度**：能写出ESKF的状态方程、传播方程、更新方程；能对比EKF和ESKF的区别（误差状态的协方差更小，线性化误差更小）；能解释MSCKF（Multi-State Constraint KF）的滑动窗口思想。

**学习时间**：2周（基于已有基础应较快）

**推荐资源**：
- 书籍：Sola, "Quaternion kinematics for the error-state Kalman filter"（ESKF必读，推导完整）
- 书籍：Barfoot, "State Estimation for Robotics"（状态估计经典，第7章EKF和第10章流形上的估计）
- 书籍：黄茨《多源融合理论与方法》（中文教材，适合快速入门）
- 论文：Mourikis & Roumeliotis, "A Multi-State Constraint Kalman Filter for Vision-Aided Inertial Navigation"（MSCKF）
- GitHub：MSCKF_VIO https://github.com/KumarRobotics/msckf_vio
- 博客：ESKF详解 https://zhuanlan.zhihu.com/p/457762975

**检验标准**：能手写一个简单的ESKF（状态：位置+速度+姿态+ba+bg），用模拟IMU数据做状态估计；能解释ESKF中为什么需要在SO(3)上做加法。

---

### 3.4 GNSS/INS组合导航与多源融合

**学什么**：GNSS定位原理（伪距、载波相位、RTK差分定位厘米级精度）。松耦合（GNSS和INS各自输出位姿再融合）、紧耦合（GNSS原始伪距/载波相位直接进入滤波器）、深耦合（INS辅助GNSS跟踪环路）。RTK/PPP-RTK的工作原理。多源融合定位架构：LiDAR + IMU + GNSS + 视觉 + 轮速计，基于因子图的统一优化框架。

**为什么学**：自动驾驶量产车定位的核心方案就是"LiDAR定位 + GNSS RTK + IMU"的多源融合。理解不同融合层次的优劣是做系统集成的基础。因子图（GTSAM）是当前工业界多源融合的主流框架。

**学到什么程度**：能区分松耦合/紧耦合/深耦合的区别和适用场景；能用GTSAM构建一个简单的LiDAR-GNSS-IMU因子图；理解RTK固定解/浮点解/单点解的区别。

**学习时间**：2周

**推荐资源**：
- 书籍：Groves, "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"（组合导航经典）
- 论文：GNSS/INS紧耦合相关综述搜索"tightly coupled GNSS INS"
- 工具：RTKLIB（开源GNSS定位软件）https://github.com/tomojitakasu/RTKLIB
- 工具：GTSAM factor graph框架 https://github.com/borglab/gtsam
- 课程：深蓝学院《组合导航》
- 博客：RTK原理 https://zhuanlan.zhihu.com/p/344117647

**检验标准**：能画出松耦合和紧耦合的系统框图；能用GTSAM构建包含IMU预积分因子、LiDAR因子、GNSS因子的因子图。

---

## Part 4: 高精地图与定位

### 4.1 高精地图构成与点云地图构建

**学什么**：高精地图（HD Map）的层次：路网层（车道级拓扑关系）、标志标线层（车道线、交通标志点位）、定位层（稠密点云/语义特征，用于匹配定位）。点云地图构建流程：多帧LiDAR点云配准 -> 去动态物体 -> 全局优化 -> 降采样存储（octree/ndt体素表示）。地图格式与存储（protobuf、OpenDRIVE）。

**为什么学**：高精地图是自动驾驶L4级方案的核心基础设施。地图构建能力是定位工程师的核心竞争力之一。

**学到什么程度**：了解高精地图各层的数据格式；能用PCL/Open3D构建小型点云地图；了解地图的更新与维护策略。

**学习时间**：1.5周

**推荐资源**：
- 书籍：《高精度地图》（杨殿阁，清华大学出版社）
- 工具：CloudCompare（点云可视化编辑工具）https://www.cloudcompare.org
- GitHub：Open3D地图构建教程
- 博客：高精地图技术解析 https://zhuanlan.zhihu.com/p/338826284

**检验标准**：能在Kitti序列上构建局部点云地图并可视化。

---

### 4.2 基于高精地图的定位与无高精地图方案

**学什么**：基于高精地图定位：将实时LiDAR点云与预建地图做NDT/ICP匹配，得到全局位姿。离线建图、在线匹配的二阶段方案。无高精地图方案：BEV语义特征匹配定位、视觉重定位（NetVLAD检索+PnP）、众包建图与地图轻量化。

**为什么学**：高精地图成本高、维护难，轻量级/无图方案是业界趋势。掌握两种方案的技术栈可灵活应对不同项目需求。

**学到什么程度**：能实现"实时点云 vs 离线地图"的NDT匹配定位；了解轻量级地图（如鸟瞰语义地图、路标地图）的设计思路。

**学习时间**：1周

**推荐资源**：
- 论文：搜索"LiDAR localization against HD map"相关论文
- 论文：Yin et al., "A Survey on Global LiDAR Localization"
- GitHub：hku-mars地图定位相关项目 https://github.com/hku-mars
- 博客：NDT定位实战 https://zhuanlan.zhihu.com/p/443128689

**检验标准**：能在已有地图上实现NDT在线定位，输出轨迹精度。

---

## Part 5: BEV感知中的定位

### 5.1 BEV视角转换与3D占据预测

**学什么**：BEV（Bird's Eye View）将多视角2D图像转换为统一的俯视图表示。核心方法：LSS（Lift-Splat-Shoot，先预测深度分布，将2D特征"提升"到3D，再"投影"到BEV）、BEVFormer（基于Transformer的空间交叉注意力，用3D参考点采样多视角特征）。3D占据预测（Occupancy Prediction）：将3D空间体素化，预测每个体素是否被占据及其语义类别。

**为什么学**：BEV感知是自动驾驶感知的主流范式。BEV空间天然适合融合定位信息，BEV特征可用于视觉定位（与高精地图BEV特征匹配）。Occupancy是比3D Box更通用的3D表示。

**学到什么程度**：理解LSS的depth distribution -> frustum feature -> voxel splatting流程；了解BEVFormer的Deformable Attention机制；能在代码层面跑通一个BEV感知demo。

**学习时间**：2周

**推荐资源**：
- 论文：Philion & Fidler, "Lift, Splat, Shoot: Encoding Images from Arbitrary Camera Rigs"
- 论文：Li et al., "BEVFormer: Learning Bird's-Eye-View Representation from Multi-Camera Images"
- 论文：Zheng et al., "Occ3D: A Large-Scale 3D Occupancy Prediction Benchmark"
- GitHub：BEVFormer https://github.com/fundamentalvision/BEVFormer
- GitHub：BEVDet https://github.com/HuangJunJie2017/BEVDet
- GitHub：OpenOccupancy https://github.com/JeffWang987/OpenOccupancy
- 课程：B站搜索"BEVFormer代码解读"

**检验标准**：能跑通BEVDet/BEVFormer在nuScenes数据集上的demo；能画出LSS的完整数据流。

---

### 5.2 BEV空间时序融合

**学什么**：将历史帧的BEV特征通过自车运动补偿（ego-motion compensation，依赖定位/里程计信息）对齐到当前帧BEV空间，拼接或注意力融合，提升单帧BEV特征的感知能力和速度估计。代表方法：BEVFormer的temporal self-attention、BEVDet4D的时序对齐策略。

**为什么学**：时序融合是BEV感知的核心能力提升手段，且天然将"定位"与"感知"耦合在一起——ego-motion的质量直接影响时序融合的效果。这在工业落地中意味着感知和定位必须联合优化。

**学到什么程度**：理解ego-motion compensation的实现（将历史BEV特征通过R,t变换到当前坐标系）；能解释时序融合如何帮助速度估计和遮挡区域感知。

**学习时间**：1周

**推荐资源**：
- 论文：BEVFormer temporal attention机制详解
- 论文：Huang et al., "BEVDet4D: Exploit Temporal Cues in Multi-camera 3D Object Detection"
- GitHub：BEVDet4D https://github.com/HuangJunJie2017/BEVDet
- 博客：BEV时序融合解析 https://zhuanlan.zhihu.com/p/557062898

**检验标准**：能解释ego-motion compensation在时序融合中的作用；能说明定位精度如何影响时序BEV融合的效果。

---

## 总体学习路径建议（约6-9个月）

| 阶段 | 时间 | 内容 |
|------|------|------|
| 第一阶段 | 月1-2 | 视觉SLAM基础（1.1-1.6）+ 点云处理基础（2.1-2.2） |
| 第二阶段 | 月3-4 | 后端优化（1.7）+ 回环检测（1.8）+ IMU预积分（3.2）+ ESKF（3.3） |
| 第三阶段 | 月5-6 | 实践阶段：ORB-SLAM3/VINS-Fusion/FAST-LIO2/LIO-SAM跑通实验（1.9, 2.3） |
| 第四阶段 | 月7-8 | 组合导航（3.4）+ 高精地图（4.1-4.2）+ BEV感知（5.1-5.2） |
| 第五阶段 | 月9 | 综合项目：在公开数据集上实现LiDAR-VIO-GNSS多源融合定位系统 |

## 核心工具链总结

| 工具 | 用途 |
|------|------|
| OpenCV | 图像处理、特征提取、PnP求解 |
| Eigen | 矩阵运算、SVD、李群/李代数 |
| PCL / Open3D | 点云处理、可视化 |
| GTSAM | 因子图优化、IMU预积分 |
| Ceres Solver | 非线性最小二乘优化 |
| ROS/ROS2 | 机器人通信框架、传感器数据采集 |
| Kalibr | 相机-IMU联合标定 |
| evo | 轨迹精度评估工具（ATE/RPE） |
| CloudCompare | 点云可视化与编辑 |
| nuScenes / Kitti / EuRoC | 公开数据集 |

## 核心论文阅读清单（按优先级排序）

1. ORB-SLAM3 (Campos et al., 2021) - 视觉SLAM集大成
2. VINS-Mono (Qin et al., 2018) - VIO标杆
3. On-Manifold Preintegration (Forster et al., 2017) - IMU预积分
4. LOAM (Zhang & Singh, 2014) - LiDAR里程计开山
5. LIO-SAM (Shan et al., 2020) - 紧耦合LiDAR-Inertial
6. FAST-LIO2 (Xu & Zhang, 2022) - 高效直接法LiDAR-Inertial
7. BEVFormer (Li et al., 2022) - BEV感知
8. Quaternion Kinematics for ESKF (Sola, 2017) - 误差状态滤波理论

---

# 模块四：3D感知与BEV感知

下面是完整的自动驾驶3D感知与BEV感知学习指南。

---

## Part 1: 点云数据处理基础

### 1.1 点云数据格式

**学什么**：了解自动驾驶中常见的点云存储格式及其内部结构。PCD（Point Cloud Data，PCL库原生格式，ASCII/Binary两种编码，头部包含FIELDS、SIZE、TYPE、WIDTH、HEIGHT等元信息）；PLY（Polygon File Format， Stanford设计，支持顶点属性和面片，常用于3D重建）；LAS（LiDAR标准格式，ASPRS制定，包含GPS时间、分类码、反射率等，常用于测绘和机载LiDAR）；bin（KITTI自定义二进制格式，每个点4个float32：x/y/z/reflectance，连续存储无头部信息，读取速度快）。

**为什么学**：自动驾驶数据管线中，原始LiDAR数据落地后第一步就是解析。不同传感器厂商（Velodyne、Hesai、Robosense、Livox）输出格式不同，KITTI用bin、nuScenes用bin+JSON元数据、Waymo用TFRecord。你需要知道每种格式的字节布局才能正确读取。

**学到什么程度**：能手写Python/C++解析每种格式；理解float32 vs float64的精度差异；了解ROSBAG中sensor_msgs::PointCloud2消息的字段布局（x/y/z/intensity/ring/time等）。

**学习时间**：2-3天。

**推荐资源**：
- 书籍：无专门书籍，参考各格式官方规范文档
- 官方文档：
  - PCD格式：`https://pointclouds.org/documentation/tutorials/pcd_file_format.html`
  - PLY格式：`http://paulbourke.net/dataformats/ply/`
  - nuScenes数据格式：`https://www.nuscenes.org/nuscenes#data-format`
- GitHub：
  - KITTI原始数据开发工具包：`https://github.com/utiasSTARS/pykitti`
  - nuScenes开发工具包：`https://github.com/nutonomy/nuscenes-devkit`
  - Waymo Open Dataset：`https://github.com/waymo-research/waymo-open-dataset`
- B站视频：搜索"KITTI数据集详解"，B站UP主"自动驾驶之心"有系统讲解

**检验标准**：能从KITTI的bin文件中读取点云并用matplotlib或Open3D可视化出完整的LiDAR扫描图；能说明nuScenes一帧sweep中包含哪些sensor token。

---

### 1.2 PCL库使用（C++）

**学什么**：PCL（Point Cloud Library）是工业界和机器人领域最成熟的点云处理C++库，版本1.13+，基于CMake构建。核心模块包括：`pcl_io`（点云读写）、`pcl_filters`（滤波）、`pcl_features`（特征提取）、`pcl_segmentation`（分割）、`pcl_registration`（配准）、`pcl_visualization`（可视化）、`pcl_kdtree`/`pcl_octree`（空间索引）。

#### 1.2.1 点云读写与可视化

**学什么**：使用`pcl::io::loadPCDFile`/`savePCDFile`读写PCD；使用`pcl::PLYReader`读写PLY；使用`pcl::visualization::PCLVisualizer`做3D可视化（设置点大小、颜色映射、坐标轴、视角控制）。

**为什么学**：自动驾驶开发中，调试点云处理管线时最频繁的操作就是"读一帧-处理-看结果"。PCLViewer是最常用的调试工具。工业界大量基于PCL开发感知模块。

**学到什么程度**：能熟练使用PCLVisualizer做点云着色（按高度、强度、类别）、多窗口对比显示、键盘回调交互。

**学习时间**：3-5天。

**推荐资源**：
- 书籍：《点云库PCL学习教程》，官方教程的中文翻译版
- 官方教程：`https://pointclouds.org/documentation/`
- GitHub：`https://github.com/PointCloudLibrary/pcl`（源码学习）
- CSDN博客：搜索"PCL点云库从入门到精通"系列

#### 1.2.2 滤波

**学什么**：
- **体素滤波（VoxelGrid）**：将3D空间划分为均匀体素网格，每个体素内用质心代替所有点。核心参数`leaf_size`控制降采样粒度。作用：将数万点降至数千点，大幅加速后续处理。
- **统计滤波（StatisticalOutlierRemoval）**：对每个点计算K近邻平均距离，剔除距离均值超过全局均值+N倍标准差的点。作用：去除飞点和噪声。
- **直通滤波（PassThrough）**：在指定轴和范围内保留点。作用：去除地面以下点、远处无效点，非常实用的预处理步骤。
- **RANSAC平面拟合/分割**：随机采样一致性算法，迭代选取最少点集拟合模型，统计内点数，保留最优模型。作用：地面分割、墙面分割的基础方法。

**为什么学**：这些是任何点云处理管线的前置步骤。自动驾驶中，一帧Velodyne VLP-16约30000点，128线机械LiDAR约150000点，不滤波无法实时处理。

**学到什么程度**：能自己调参完成完整的预处理管线：直通滤波去除远点 -> 体素降采样 -> 统计滤波去飞点 -> RANSAC地面分割。理解每个参数对结果的影响。

**学习时间**：5-7天。

**推荐资源**：
- 官方教程（每种滤波器都有独立教程页）：`https://pointclouds.org/documentation/group__filters.html`
- 书籍：《点云库PCL学习教程》第6章
- B站：搜索"PCL滤波"，有大量实操教程

#### 1.2.3 分割

**学什么**：
- **欧氏聚类（EuclideanClusterExtraction）**：基于KD-Tree的BFS/DFS聚类，距离阈值内的点归为同一类。参数：`cluster_tolerance`（邻域半径）、`min_cluster_size`/`max_cluster_size`。作用：3D检测后处理中的经典方法，将前景点聚类为独立物体。
- **区域生长分割（RegionGrowing）**：从种子点出发，根据法线相似性和曲率扩展区域。适合分割具有光滑表面的物体。
- **平面分割（SACSegmentation + Planar Model）**：基于RANSAC拟合平面模型，配合`ExtractIndices`提取平面内/外点。

**为什么学**：欧氏聚类是传统3D感知pipeline中物体实例分割的核心步骤（如Apollo中的点云聚类模块）。理解这些方法有助于理解深度学习方法的改进动机。

**学到什么程度**：能完成"地面去除 -> 障碍物点提取 -> 欧氏聚类 -> 包围盒生成"的完整流程。

**学习时间**：5-7天。

**推荐资源**：
- 官方教程：`https://pointclouds.org/documentation/tutorials/cluster_extraction.html`
- Apollo源码：`https://github.com/ApolloAuto/apollo`（modules/lidar/point_pillars/）

#### 1.2.4 特征估计

**学什么**：
- **法线估计（NormalEstimation）**：对每个点拟合局部切平面，用最小二乘法求法线方向。核心参数K（近邻数）或搜索半径。
- **FPFH（Fast Point Feature Histograms）**：33维特征向量，编码每个点与其邻域点之间的法线角度关系。是点特征描述子的经典方法，广泛用于点云配准（与SAC-IA结合）。

**为什么学**：法线是几乎所有点云表面分析的基础；FPFH是理解"如何用固定长度向量描述一个点的局部几何"的入门，这是PointNet等深度学习方法要解决的核心问题。

**学到什么程度**：能计算并可视化法线场；能用FPFH做简单的点云粗配准。

**学习时间**：3-5天。

**推荐资源**：
- 官方教程：`https://pointclouds.org/documentation/tutorials/normal_estimation.html`
- 官方教程：`https://pointclouds.org/documentation/tutorials/fpfh_estimation.html`
- 论文：Rusu et al., "Fast Point Feature Histograms (FPFH) for 3D Registration", ICRA 2009

---

### 1.3 Open3D使用（Python）

**学什么**：Open3D是Intel开源的现代3D数据处理库（Python/C++），API设计简洁，可视化效果好（基于OpenGL），支持Jupyter notebook内嵌可视化。核心功能包括：点云读写（`o3d.io.read_point_cloud`）、滤波（`voxel_down_sample`、`uniform_down_sample`、`statistical_outlier_removal`）、法线估计、ICP配准（`registration_icp`，支持point-to-point和point-to-plane）、彩色点云处理、TSDF融合、3D重建（Poisson/Ball-Pivoting）。

**为什么学**：学术界实验和论文复现首选Python工具，快速原型开发效率远高于PCL+C++。大部分3D深度学习论文的demo代码都用Open3D。

**学到什么程度**：能用Open3D完成点云可视化、ICP配准、体素下采样、法线计算。能在Jupyter Notebook中交互式查看3D点云。

**学习时间**：3-5天（配合PCL学习可缩短）。

**推荐资源**：
- 官方教程：`http://www.open3d.org/docs/release/tutorial/`
- GitHub：`https://github.com/isl-org/Open3D`（star数11k+）
- B站：搜索"Open3D点云处理"，有系统教程
- 书籍：Open3D官方教程本身就是最好的教材

**检验标准**：能用Open3D加载KITTI点云，做体素降采样、法线估计、ICP配准两帧点云并可视化配准结果。

---

### 1.4 点云体素化（Voxelization）原理与实现

**学什么**：体素化是将无序点云映射到规则3D网格的核心操作。具体包括：
- **占位体素化（Occupancy Voxelization）**：包含点的体素标记为1，否则为0，得到稀疏3D二值张量。
- **特征体素化（Feature Voxelization）**：每个体素内记录点的统计特征（均值坐标、强度均值/最大值、点数等）。这是VoxelNet/SECOND/PointPillars等方法的核心预处理。
- **体素化实现细节**：坐标归一化（`floor((coord - min_coord) / voxel_size)`得到体素索引）、哈希映射（字典/unordered_map，key为体素索引的编码）、动态体素vs固定最大体素数、点到体素索引的反向映射。
- **稀疏体素表示**：只存储非空体素，用字典/哈希表索引，节省内存。

**为什么学**：体素化是所有Voxel-based 3D检测方法的起始步骤。理解其细节（如何处理边界、如何聚合多点、如何保证可微性）直接关系到你能否理解后续VoxelNet/SECOND/PointPillars的实现。

**学到什么程度**：能手写Python实现体素化（numpy版本）；理解`spconv`库中`SparseConvTensor`的indices/features格式；能解释为什么需要pillars/z-voxels两种不同粒度。

**学习时间**：3-5天。

**推荐资源**：
- 论文：VoxelNet (Zhou et al., 2018) 的Section 3.1 Feature Learning Network
- GitHub：`https://github.com/traveller59/spconv`（稀疏卷积库，内含体素化实现）
- 源码：MMDetection3D中的`voxel/voxel_generator.py`

**检验标准**：能手写体素化函数，输入N×4点云，输出体素坐标和体素内特征；能解释pillars（柱状体素）和普通体素的区别。

---

### 1.5 点云在BEV/柱坐标下的表示

**学什么**：
- **笛卡尔坐标BEV投影**：将3D点的(x, y)映射到2D BEV网格（bird's-eye view），z轴信息通过高度编码（z-bin、统计值）保留在channel维度。BEV网格本质上是一个[C, H, W]的伪图像。
- **柱坐标表示（Cylindrical）**：将笛卡尔坐标(x, y, z)转换为(r, theta, z)，r为径向距离，theta为方位角，z为高度。Cylinder3D等工作使用此表示，优势是与LiDAR扫描模式天然匹配（线束环形分布），在远距离区域点更稠密时分辨率更合理。
- **Spherical坐标表示**：(r, theta, phi)，部分工作用于Range View（距离图）表示。
- **Range View投影**：将3D点投影到2D距离图[u, v]上，u对应水平方向的beam index，v对应垂直方向的ring index，像素值为距离/强度/深度。

**为什么学**：BEV表示是所有BEV感知方法的基础。理解不同坐标系下点云表示的优劣（笛卡尔BEV近处稠密远处稀疏；柱坐标更均匀；Range View信息无损但畸变严重）是选择感知方案的前提。

**学到什么程度**：能将KITTI点云分别投影到笛卡尔BEV图和Range View图并可视化；理解BEV分辨率（如0.1m/pixel vs 0.5m/pixel）对检测精度和计算量的影响。

**学习时间**：2-3天。

**推荐资源**：
- 论文：Cylinder3D (Zhu et al., CVPR 2021) 的方法部分
- GitHub：`https://github.com/xinge008/Cylinder3D`
- 博客：知乎搜索"自动驾驶中的BEV表示"，有多种表示方法的对比图

**检验标准**：能手写Python代码将KITTI点云投影为BEV高度图和密度图，并解释为什么BEV表示避免了透视投影的遮挡问题。

---

## Part 2: 3D目标检测 - Point-based方法

### 2.1 PointNet

**学什么**：
- **问题定义**：如何直接从无序点集学习特征？核心挑战是点集的排列不变性（permutation invariance）。
- **对称函数设计**：论文核心思想——用对称函数保证排列不变性。具体实现为：对每个点独立MLP提升到高维 -> Max Pooling（沿点维度取最大值） -> 得到全局特征向量。Max Pooling就是那个对称函数。
- **网络结构**：Input Transform (3x3 T-Net) -> shared MLP(64,64) -> Feature Transform (64x64 T-Net) -> shared MLP(64,128,1024) -> Max Pool -> 全局1024维特征。
- **T-Net（Spatial Transformer Network思想）**：学习输入点云/特征的对齐变换矩阵，解决坐标系不一致问题。
- **应用**：分类（直接用全局特征接MLP）和分割（全局特征拼接回每个点的局部特征，再接MLP）。

**为什么学**：PointNet是点云深度学习的开山之作（CVPR 2017），其"对称函数"思想深刻影响了后续所有工作。理解它是理解PointNet++、PointRCNN等的基础。面试高频考点。

**学到什么程度**：能手写PointNet的PyTorch实现（分类和分割版本）；能解释为什么Max Pooling能实现排列不变性，以及为什么不能直接用全连接层处理点云。

**学习时间**：5-7天。

**推荐资源**：
- 论文：Qi et al., "PointNet: Deep Learning on Point Sets for 3D Classification and Segmentation", CVPR 2017
- 原始代码：`https://github.com/charlesq34/pointnet`
- PyTorch复现：`https://github.com/yanx27/Pointnet_Pointnet2_pytorch`（含中文注释）
- B站视频：搜索"PointNet论文精读"，推荐"跟李沐学AI"的论文精读系列
- 知乎：搜索"PointNet详解"，有多篇图文并茂的解读

**检验标准**：能不看代码手写PointNet核心forward过程；能画出网络结构图并解释T-Net的作用；能在ModelNet40上跑通训练并复现约89%的分类准确率。

---

### 2.2 PointNet++

**学什么**：
- **PointNet的局限**：PointNet对每个点独立处理再全局池化，缺乏局部上下文（local context）捕获能力，无法学习多尺度几何模式。
- **Set Abstraction（SA）模块**：核心创新。三层结构：(1) FPS最远点采样选中心点 -> (2) Ball Query或KNN找局部邻域 -> (3) 对每个邻域用mini-PointNet提取局部特征。逐层SA扩大感受野，形成层次化特征。
- **多尺度分组（MSG）和多分辨率分组（MRG）**：MSG对同一中心点用不同半径的Ball Query捕获多尺度信息（计算量大）；MRG融合不同层级的特征（更高效）。
- **特征传播（Feature Propagation）**：用于分割任务的上采样模块。通过逆距离加权插值将稀疏层特征传播回密集层，再拼接skip-connection特征并MLP处理。

**为什么学**：PointNet++是层次化点云特征学习的奠基工作，后续几乎所有Point-based检测方法（PointRCNN、3DSSD等）都使用SA模块作为backbone。

**学到什么程度**：能手写SA模块（含FPS + Ball Query + mini-PointNet）和FP模块的PyTorch实现；理解FPS vs 随机采样 vs 关键点采样的区别。

**学习时间**：7-10天。

**推荐资源**：
- 论文：Qi et al., "Deep Hierarchical Feature Learning on Point Sets in a Metric Space", NeurIPS 2017
- 原始代码：`https://github.com/charlesq34/pointnet2`
- PyTorch复现（推荐）：`https://github.com/yanx27/Pointnet_Pointnet2_pytorch`
- CUDA加速版：`https://github.com/sshaoshuai/Pointnet2.PyTorch`（SSHA版，含CUDA的FPS/Query/KNN算子，工业级实现）
- B站：搜索"PointNet++详解"，"3D视觉工坊"有系列讲解

**检验标准**：能在ShapeNet上跑通分割任务；能画出SA模块和FP模块的结构图并手推数据维度变化。

---

### 2.3 PointRCNN

**学什么**：
- **两阶段架构**：Stage 1（RPN）基于PointNet++ backbone生成3D proposal；Stage 2（RCNN）对每个proposal做精化。
- **Stage 1 - 前景分割 + proposal生成**：PointNet++ backbone提取逐点特征 -> 逐点预测前景/背景 + 3D bbox（中心、尺寸、朝向） -> NMS去重得到proposal。
- **Stage 2 - RoI-aware特征池化**：对每个proposal内部的点做坐标归一化（变换到proposal局部坐标系），再PointNet聚合特征，预测bbox残差精化。
- **Canonical坐标变换**：将proposal内的点变换到以proposal中心为原点、朝向对齐的局部坐标系，这是Stage 2的核心创新。

**为什么学**：PointRCNN是第一个真正有效的纯点云两阶段3D检测方法，其"先分割后回归"的思路和canonical坐标变换影响了后续大量工作（Part-A2、PV-RCNN等）。

**学到什么程度**：理解两阶段设计的动机和优势；能跑通PointRCNN在KITTI上的训练和评估；理解3D IoU计算和NMS。

**学习时间**：7-10天。

**推荐资源**：
- 论文：Shi et al., "PointRCNN: 3D Object Proposal Generation and Detection from Point Cloud", CVPR 2019
- 原始代码（PyTorch）：`https://github.com/sshaoshuai/PointRCNN`
- 讲解：知乎"PointRCNN详解"（搜索高赞文章）

**检验标准**：能在KITTI上运行PointRCNN，理解Easy/Moderate/Hard三个难度级别的评估标准。

---

### 2.4 3DSSD

**学什么**：
- **采样策略改进**：提出F-FPS（Feature-FPS），结合欧氏距离和特征距离做最远点采样，相比FPS更好地保留前景点。
- **单阶段设计**：去除PointRCNN的第二阶段，通过更好的采样和回归策略实现单阶段高精度检测。
- **Candidate Generation层**：对采样点做前景/背景分类和box回归。
- **3D中心性标签分配**：用物体3D中心的投影点做正样本分配，比IoU分配更稳定。

**为什么学**：了解如何通过精心设计采样策略来提升单阶段方法的性能，理解"采样什么点"比"网络多深"更重要的设计哲学。

**学到什么程度**：能对比F-FPS和FPS的采样结果差异；理解单阶段vs两阶段的精度-速度权衡。

**学习时间**：5-7天。

**推荐资源**：
- 论文：Yang et al., "3DSSD: Point-based 3D Single Stage Object Detector", CVPR 2020
- 代码：`https://github.com/Jia-Research-Lab/3DSSD`

---

### 2.5 Part-A2 和 PV-RCNN

**学什么**：
- **Part-A2**（Part-Aware Anchor-free 3D检测）：(1) 前景点分割 + 逐点预测物体part位置（8个角点 + 中心）；(2) 基于预测part位置做anchor-free proposal生成；(3) RoI-aware点云池化做精化。
- **PV-RCNN**（Point-Voxel Feature Set Abstraction）：融合Point-based和Voxel-based两种表示的优势。(1) Voxel backbone（3D稀疏卷积）提取体素特征；(2) VSA（Voxel Set Abstraction）模块将关键点（FPS采样）周围的体素特征聚合到关键点上；(3) RoI-grid pooling对proposal区域内的关键点做进一步聚合；(4) 两阶段精化。
- **关键点（Keypoints）的使用**：PV-RCNN用8192个FPS关键点作为信息聚合的枢纽，连接稀疏体素特征和逐点特征。

**为什么学**：PV-RCNN是KITTI排行榜上的经典方法，其"关键点作为信息枢纽"的思想非常优雅，是理解后续PV-RCNN++、Voxel-RCNN等系列工作的基础。Part-A2展示了"先预测部件再回归整体"的设计思路。

**学到什么程度**：能画出PV-RCNN的完整pipeline（VSA、RoI-grid pooling）；理解Point-based和Voxel-based方法各自的优缺点及融合动机。

**学习时间**：7-10天（PV-RCNN较复杂）。

**推荐资源**：
- Part-A2 论文：Shi et al., "From Points to Parts: 3D Object Detection from Point Cloud with Part-aware and Part-aggregation Network", TPAMI 2021
- PV-RCNN 论文：Shi et al., "PV-RCNN: Point-Voxel Feature Set Abstraction for 3D Object Detection", CVPR 2020
- 代码：`https://github.com/sshaoshuai/OpenPCDet`（OpenPCDet中包含PV-RCNN、Part-A2的实现，必读代码）
- 讲解：知乎搜索"PV-RCNN详解"

**检验标准**：能在OpenPCDet中跑通PV-RCNN在KITTI上的训练；能画出VSA模块的示意图。

---

## Part 3: 3D目标检测 - Voxel-based方法

### 3.1 VoxelNet

**学什么**：
- **核心思路**：将点云体素化 -> 对每个体素内的点用VFE（Voxel Feature Encoding）层提取特征 -> 送入3D卷积和2D RPN。
- **VFE层**：对体素内每个点先MLP提升维度 -> Max Pooling得到体素级特征 -> 拼接回逐点特征。本质是mini-PointNet。
- **3D卷积**：对体素特征张量做3D卷积（Cin x D x H x W），逐步降低高度维度，最后压缩为2D BEV特征图。
- **RPN**：2D Region Proposal Network，与Faster R-CNN的RPN类似，在BEV特征图上生成anchor并预测。
- **效率问题**：3D卷积计算量大，尤其在高分辨率体素下。VoxelNet的训练需要约1块V100跑数天。

**为什么学**：VoxelNet是第一个端到端学习体素特征的3D检测方法，其"体素化+3D卷积"的范式奠定了Voxel-based方法的基础。理解其效率瓶颈是理解后续SECOND、PointPillars改进动机的关键。

**学到什么程度**：理解VFE的实现细节；能解释为什么3D卷积在自动驾驶场景下效率低（大部分体素为空）。

**学习时间**：5-7天。

**推荐资源**：
- 论文：Zhou et al., "VoxelNet: End-to-End Learning for Point Cloud Based 3D Object Detection", CVPR 2018
- 解读：知乎"VoxelNet论文精读"

---

### 3.2 SECOND

**学什么**：
- **稀疏3D卷积（Sparse 3D Convolution）**：SECOND的核心贡献。只对非空体素做卷积计算（而非遍历全部体素），用哈希表管理稀疏索引，计算量从O(D x H x W)降到O(N_voxel)。这是工业界3D检测的标配技术。
- **网络结构**：体素化 -> 稀疏3D卷积层 -> 稀疏到密集转换（SparseToDense） -> 2D卷积（BEV backbone） -> 检测头。
- **Anchor设计**：在BEV上放置不同朝向、不同尺寸的anchor box。
- **Loss设计**：分类用focal loss，回归用smooth-L1，加上方向分类loss。

**为什么学**：SECOND的稀疏卷积使得3D卷积从"学术可行但工程不可用"变成"又快又好"。`spconv`库已成为工业界标准。面试中"稀疏卷积"是高频技术问题。

**学到什么程度**：理解`spconv.SparseConv3d`的输入输出格式（indices + features）；能在OpenPCDet中跑通SECOND在KITTI上的训练。

**学习时间**：5-7天。

**推荐资源**：
- 论文：Yan et al., "SECOND: Sparsely Embedded Convolutional Detection", Sensors 2018
- spconv库：`https://github.com/traveller59/spconv`（必看，理解SparseConvTensor）
- spconv v2文档：`https://github.com/traveller59/spconv`（新版支持更多backbone）
- OpenPCDet中SECOND的config和实现

**检验标准**：能在OpenPCDet中训练SECOND并达到KITTI Moderate级别约78+ AP（Car类）；能解释稀疏卷积相比密集卷积的速度提升原理。

---

### 3.3 PointPillars（精学）

**学什么**：
- **Pillar编码**：将3D空间在x-y平面划分为网格（每个网格就是一个pillar/柱子），z方向不切分，每个柱子内所有点聚合为一个固定长度的特征向量。相比体素化大幅减少3D维度。
- **PointNet风格的柱内特征提取**：对柱内N个点：(x,y,z,r,x_c,y_c,z_c,x_p,y_p)，其中下标c是到柱内均值的偏移，下标p是到柱中心的偏移 -> MLP -> Max Pooling -> 得到每个pillar的[C]特征。
- **伪图像（Pseudo Image）生成**：将非空pillar的特征散布到2D网格上，形成[C, H, W]的伪图像，直接送入2D卷积backbone（如SECOND的2D部分或ResNet）。
- **2D检测头**：SSD风格的检测头，在不同尺度的feature map上预测。
- **为什么工业界最爱**：没有3D卷积 -> 推理速度快（在TensorRT上可达60+ FPS）；pillars设计避免了z维度的计算；结果是纯2D卷积网络，部署优化成熟。

**为什么学**：PointPillars是工业界使用最广泛的3D检测方法之一。Apollo、Autoware等开源自动驾驶平台都集成了PointPillars。其简洁性和速度使其成为量产落地的首选。面试必问。

**学到什么程度**：必须理解到代码级别。能手写Pillar编码器；能在MMDetection3D中自定义PointPillars配置并训练；能解释pillar_size的选择对精度和速度的影响。

**学习时间**：10-14天（精学，含代码实现）。

**推荐资源**：
- 论文：Lang et al., "PointPillars: Fast Encoders for Object Detection from Point Clouds", CVPR 2019
- 代码（OpenPCDet）：`https://github.com/sshaoshuai/OpenPCDet`（`pcdet/models/backbones_3d/pointpillar_scatter.py`）
- 代码（MMDetection3D）：`https://github.com/open-mmlab/mmdetection3d`
- 原始代码（可选）：`https://github.com/traveller59/second.pytorch`
- 博客：知乎"PointPillars详解"有多篇高质量文章
- 视频：B站搜索"PointPillars代码详解"

**检验标准**：
1. 能画出PointPillars的完整网络结构图（Pillar Encoder -> Pseudo Image -> 2D Backbone -> Detection Head）。
2. 能手推Pillar Encoding过程：给定一组点，手动计算pillar内特征。
3. 能在MMDetection3D/OpenPCDet中训练并在KITTI上达到SOTA水平（Car Moderate ~77 AP）。
4. 能解释pillar_size从0.16到0.4变化时精度和速度如何变化。

---

### 3.4 CenterPoint

**学什么**：
- **Center-based表示**：不使用anchor box，而是在BEV特征图的每个物体3D中心位置预测heatmap、尺寸、朝向、速度、偏移等。继承自2D检测中的CenterNet思想。
- **骨干网络**：VoxelNet/SECOND backbone -> BEV特征图 -> CenterHead。
- **CenterHead**：多个并行的卷积头，分别预测heatmap（高斯热图，物体中心处为峰值）、3D尺寸、子中心偏移、z坐标、朝向（sin/cos编码）、速度。
- **标签分配**：无需anchor匹配，直接用GT中心在heatmap上绘制高斯圆作为正样本。
- **两阶段精化（CenterPoint-Two-Stage）**：第一阶段用CenterHead生成粗检测 -> 第二阶段对每个检测框内部点做特征池化 -> 精化尺寸、朝向、速度。
- **速度估计**：通过连续帧之间的物体中心匹配来估计速度，这是nuScenes评估的关键指标。

**为什么学**：CenterPoint是nuScenes LiDAR检测排行榜的主流方法，也是多模态融合方法（BEVFusion等）常用的LiDAR分支。anchor-free设计消除了繁琐的anchor超参调优。工业界多个量产方案采用CenterPoint作为LiDAR检测模块。

**学到什么程度**：能理解Center Head的多头输出设计；能在MMDetection3D中配置并训练CenterPoint；理解NMS-free的设计（通过max pooling做peak selection替代NMS）。

**学习时间**：7-10天。

**推荐资源**：
- 论文：Yin et al., "Center-based 3D Object Detection and Tracking", CVPR 2021
- 代码：`https://github.com/tianweiy/CenterPoint`（原始实现，mmdet3d风格）
- MMDetection3D中的CenterPoint config：`configs/centerpoint/`
- 解读：知乎"CenterPoint详解"

**检验标准**：能在nuScenes上训练CenterPoint并理解NDS评估指标的含义（mAP、mATE、mASE、mAOE、mAVE、mAAE）。

---

## Part 4: 3D目标检测 - Multi-view/BEV方法

### 4.1 相机3D检测基础

#### 4.1.1 深度估计

**学什么**：
- **单目深度估计**：从单张图像预测每个像素的深度。方法包括：(a) 有监督方法，用LiDAR深度做监督（如DORN、BTS）；(b) 自监督方法，用相邻帧的光度一致性做自监督（Monodepth系列）；(c) 基础模型，如Depth Anything、MiDaS等大模型。
- **双目深度估计**：利用两个相机之间的基线和视差计算深度。核心：立体匹配（Stereo Matching）-> 视差图 -> 深度 = f*baseline/disparity。代表方法：PSMNet、GA-Net、AANet、RAFT-Stereo。
- **多目深度估计**：利用多个环绕相机的重叠区域做多视角立体匹配。代表方法：MVSNet系列、SurroundDepth。
- **深度估计在BEV感知中的角色**：LSS等方法显式预测深度分布；BEVDepth用LiDAR监督深度估计网络。

**为什么学**：纯视觉3D检测的核心瓶颈就是深度估计。深度估计质量直接决定BEV特征的质量。面试中"如何从2D图像获得3D信息"是必答题。

**学到什么程度**：理解深度估计的不同范式及其误差特性；能在KITTI上跑通一个单目深度估计模型。

**学习时间**：7-10天。

**推荐资源**：
- 单目深度（自监督）：Monodepth2 论文 + 代码 `https://github.com/nianticlabs/monodepth2`
- 单目深度（基础模型）：Depth Anything V2 `https://github.com/DepthAnything/Depth-Anything-V2`
- 双目深度：PSMNet `https://github.com/JiaRenChang/PSMNet`
- 课程：Stanford CS231A（计算机视觉），深度估计相关章节

#### 4.1.2 2D-to-3D提升方法

**学什么**：
- **逆透视映射（IPM/Inverse Perspective Mapping）**：假设地面平面，将2D图像变换到BEV。简单但受限于平面假设。
- **基于深度分布的提升（Depth Distribution-based）**：为每个像素预测离散深度分布，沿射线方向"撒"特征，生成3D特征体。这是LSS的核心思想。
- **基于Transformer的提升**：用cross-attention让BEV查询在图像特征上做注意力，隐式学习2D到3D的映射。这是BEVFormer的思路。
- **基于MLP的提升**：用MLP学习从2D像素坐标+特征到3D空间位置的映射（如ImVoxelNet）。

**为什么学**：2D-to-3D提升是纯视觉BEV感知的核心技术。不同提升方式各有优劣，理解它们有助于选择和设计感知方案。

**学到什么程度**：能解释LSS的depth distribution + outer product的数学原理；能对比IPM、LSS、Transformer-based三种方法的适用场景和局限。

**学习时间**：5-7天。

**推荐资源**：
- LSS论文（见4.2.1）
- IPM教程：搜索"IPM逆透视变换详解"
- 知乎："BEV感知中的2D到3D提升方法综述"

---

### 4.2 BEV感知

#### 4.2.1 LSS（Lift, Splat, Shoot）

**学什么**：
- **Lift**：对每个图像像素，用深度网络预测离散深度分布（D维softmax），然后将2D特征沿深度方向外积（outer product），得到每个像素对应的D个3D点特征。数学上：`feature_3d = depth_distribution @ image_feature`（矩阵乘），得到[D, H, W, C]的伪3D特征。
- **Splat**：通过相机内外参将这些3D点投影到BEV网格（用cumsum trick实现可微的scatter操作），累加到对应的BEV cell中。
- **Shoot**：在BEV特征图上接下游任务头（如检测、分割）。
- **核心创新**：可微的depth-based lifting + 2D-to-3D投影，无需显式3D操作。

**为什么学**：LSS是BEV感知的开山之作，后续BEVDet、BEVDepth、BEVStereo等大量工作都基于LSS框架。理解LSS是理解整个BEV感知方向的前提。

**学到什么程度**：能画出LSS的完整pipeline图（图像 -> backbone -> depth net -> lift -> splat -> BEV feature）；能手推depth distribution与image feature的外积操作的维度变化；理解cumsum trick的可微性。

**学习时间**：7-10天。

**推荐资源**：
- 论文：Philion & Fidler, "Lift, Splat, Shoot: Encoding Images from Arbitrary Camera Rigs by Implicitly Unprojecting to 3D", ECCV 2020
- 代码：`https://github.com/nv-tlabs/lift-splat-shoot`（原始实现）
- 代码：`https://github.com/HuangJunJie27/BEV.SGD`（更易读的复现）
- B站视频：搜索"LSS论文精读"或"BEV感知LSS"

**检验标准**：能解释为什么LSS用"depth distribution"而不是单一深度值；能说明cumsum trick如何替代了不可微的scatter_add操作。

---

#### 4.2.2 BEVDet 和 BEVDet4D

**学什么**：
- **BEVDet**：在LSS基础上的工程化改进。(1) 数据增强：图像增强后需要同步更新相机参数（BEV augmentation）；(2) BEV Encoder：在BEV特征图上加额外的卷积层（如ResNet block）增强BEV特征；(3) 检测头：CenterPoint风格的anchor-free检测头。
- **BEVDet4D**：引入时序信息。将历史帧的BEV特征通过ego-motion补偿（用自车pose将历史BEV对齐到当前帧）后拼接/融合，显著提升速度估计和远距离检测性能。
- **时序对齐（Temporal Alignment）**：用自车的里程计pose将t-1时刻的BEV特征warp到t时刻的坐标系下，这是BEV感知中时序融合的标准做法。

**为什么学**：BEVDet是第一个将LSS方法工程化并达到有竞争力结果的工作；BEVDet4D的时序融合方案成为后续所有BEV方法的标准模块。

**学到什么程度**：理解BEVDet的数据增强pipeline（特别是BEV空间的增强）；理解时序BEV特征对齐的实现。

**学习时间**：5-7天。

**推荐资源**：
- BEVDet论文：Huang et al., "BEVDet: High-Performance Multi-Camera 3D Object Detection in Bird-Eye-View", 2021
- BEVDet4D论文：Huang et al., "BEVDet4D: Exploit Temporal Cues in Multi-camera 3D Object Detection", 2022
- 代码：`https://github.com/HuangJunJie27/BEVDet`
- 讲解：知乎"BEVDet系列详解"

---

#### 4.2.3 BEVDepth

**学什么**：
- **核心问题**：LSS中的深度估计只用图像监督（如投影深度），质量差，是BEV感知的瓶颈。
- **显式深度监督**：BEVDepth用LiDAR点云投影到图像平面获得GT深度，直接监督深度估计网络。这大幅提升了深度估计质量。
- **深度估计网络设计**：使用深度估计专用的encoder-decoder网络，输出D维离散深度分布。
- **Camera-aware Depth Estimation**：考虑不同相机的内参差异，对深度预测做相机特定的校准。
- **效率优化**：CUDA实现的高效hardvoxel池化，加速Lift+Splat过程。

**为什么学**：BEVDepth证明了深度估计质量是纯视觉BEV感知的核心瓶颈，引入LiDAR监督后性能大幅提升。这是"如何提升BEV感知质量"的关键思路。

**学到什么程度**：理解深度监督信号的生成方式（LiDAR投影 -> GT深度图）；理解为什么深度监督对BEV性能有如此大的提升。

**学习时间**：5-7天。

**推荐资源**：
- 论文：Li et al., "BEVDepth: Acquisition of Reliable Depth for Multi-view 3D Object Detection", AAAI 2023
- 代码：`https://github.com/Megvii-BaseDetection/BEVDepth`
- MMDetection3D中BEVDepth的config

---

#### 4.2.4 BEVFormer

**学学什么**：
- **Transformer-based BEV构建**：不用LSS的显式深度估计，而是定义一组可学习的BEV queries（[H*W, C]的网格），通过cross-attention从多视角图像特征中聚合信息。
- **Spatial Cross-Attention**：每个BEV query对应3D空间中的一条柱状体（pillar），将柱状体均匀采样若干3D参考点，通过相机参数投影到各相机的图像平面，在对应位置采样图像特征做deformable attention。
- **Temporal Self-Attention**：在时间维度上，当前帧BEV query与上一帧BEV特征做self-attention，实现时序信息融合。
- **Encoder结构**：多个Transformer encoder layer交替做Spatial Cross-Attention和Temporal Self-Attention。
- **3D检测头**：BEV特征图上接Deformable DETR风格的检测头。

**为什么学**：BEVFormer是第一个用Transformer构建BEV表示并达到SOTA性能的方法。其"query-based"的BEV构建方式与LSS的"depth-based"方式形成两大范式。理解BEVFormer对后续UniAD、VAD等端到端自动驾驶方案至关重要。

**学到什么程度**：能画出Spatial Cross-Attention的完整流程（BEV query -> 3D参考点 -> 2D投影 -> deformable attention）；理解为什么Transformer-based方法不需要显式深度估计。

**学习时间**：10-14天。

**推荐资源**：
- 论文：Li et al., "BEVFormer: Learning Bird's-Eye-View Representation from Multi-Camera Images via Spatiotemporal Transformers", ECCV 2022
- 代码：`https://github.com/fundamentalvision/BEVFormer`（基于mmdetection3d）
- 视频讲解：B站搜索"BEVFormer论文精读"
- 前置知识：Deformable DETR论文和代码（`https://github.com/fundamentalvision/Deformable-DETR`）

**检验标准**：能解释Spatial Cross-Attention中参考点的生成过程；能对比LSS-based和Transformer-based两种BEV构建方式的优劣。

---

#### 4.2.5 StreamPETR

**学什么**：
- **流式BEV感知**：不同于BEVFormer的逐帧重建BEV，StreamPETR用一组object queries在时序上持续传递信息（类似tracking-by-detection），实现更高效的时序融合。
- **运动感知层传播（Motion-aware Layer Normalization）**：用ego-motion将历史帧的query位置变换到当前帧坐标系，通过position embedding注入时序信息。
- **Memory Queue**：保存历史帧的query特征和位置信息，与当前帧query做attention。
- **优势**：推理延迟低（不需要每帧重新构建完整BEV），适合实时自动驾驶系统。

**为什么学**：StreamPETR代表了BEV感知从"逐帧重建BEV"到"流式维护状态"的演进方向。这种设计更适合实际部署。

**学到什么程度**：理解query-based时序融合与BEV-based时序融合的区别；能在mmdet3d中跑通StreamPETR。

**学习时间**：5-7天。

**推荐资源**：
- 论文：Wang et al., "Exploring Object-Centric Temporal Modeling for Efficient Multi-Frame 3D Object Detection", ICCV 2023
- 代码：`https://github.com/exiawsh/StreamPETR`

---

### 4.3 纯视觉 vs LiDAR vs 多模态融合

#### 4.3.1 融合范式

**学什么**：
- **前融合（Early Fusion）**：在原始数据层融合，如PointPainting将图像语义标签"涂"到点云上。
- **后融合（Late Fusion）**：各模态独立检测后融合结果（如NMS合并、投票）。
- **特征级融合（Feature-level Fusion）**：在特征空间融合，如BEVFusion。
- **各范式的权衡**：前融合信息保留充分但耦合度高；后融合灵活但信息利用不充分；特征级融合是主流方向。

**为什么学**：多模态融合是自动驾驶感知的核心技术路线。纯LiDAR成本高但精度好，纯视觉成本低但深度不准，融合两者是工业界的主流方案。

**学到什么程度**：能解释三种融合范式的原理和优缺点；理解为什么特征级融合（尤其是BEV空间融合）是当前主流。

**学习时间**：3-5天。

#### 4.3.2 PointPainting / PointAugmenting

**学什么**：
- **PointPainting**：用2D语义分割网络对图像做分割 -> 将每个相机像素的语义score投影到对应的LiDAR点上 -> 将语义通道拼接到点云原始特征后 -> 送入LiDAR 3D检测器。
- **PointAugmenting**：类似PointPainting，但拼接的是CNN提取的图像深度特征（而非语义标签），信息更丰富。

**为什么学**：前融合的代表方法，简单有效，理解它有助于理解"为什么后融合不够好"和"信息应该在哪个阶段融合"。

**学习时间**：3天。

**推荐资源**：
- 论文：Vora et al., "PointPainting: Sequential Fusion for 3D Object Detection", CVPR 2020
- 论文：Wang et al., "PointAugmenting: Cross-Modal Augmentation for 3D Object Detection", CVPR 2021

#### 4.3.3 BEVFusion

**学什么**：
- **MIT版BEVFusion**：统一的多模态BEV融合框架。(1) LiDAR分支：VoxelNet backbone -> BEV特征图；(2) Camera分支：图像backbone -> LSS提升到BEV特征图；(3) 在BEV空间拼接或相加两个分支的BEV特征 -> 共享BEV backbone + 检测头。
- **高效BEV Pooling**：CUDA实现的高效相机特征到BEV的映射，解决LSS Splat步骤的效率瓶颈。
- **多任务**：同时做3D检测和BEV分割。
- **性能**：在nuScenes上达到68.5 NDS，首次证明camera和LiDAR BEV特征的有效融合。

**为什么学**：BEVFusion是多模态BEV融合的经典之作，是后续众多多模态工作的baseline。理解其架构有助于理解整个多模态感知的技术路线。

**学到什么程度**：能画出BEVFusion的双分支架构图；能在MMDetection3D中跑通BEVFusion的训练。

**学习时间**：7-10天。

**推荐资源**：
- 论文：Liu et al., "BEVFusion: Multi-Task Multi-Sensor Fusion with Unified Bird's-Eye View Representation", ICRA 2023
- MIT代码：`https://github.com/mit-han-lab/bevfusion`（基于mmdet3d）
- 解读：知乎"BEVFusion详解"

#### 4.3.4 TransFusion

**学什么**：
- **Transformer-based LiDAR-Camera融合**：用LiDAR检测的预测中心点初始化object queries，通过cross-attention从图像特征中提取对应位置的视觉信息，实现查询驱动的多模态融合。
- **Soft Association**：不需要显式的点到像素的投影对应关系（如PointPainting需要相机标定），通过attention机制自动学习对应关系。
- **优势**：对相机标定误差鲁棒；不需要精确的传感器标定。

**为什么学**：展示了如何用Transformer做灵活的多模态融合，不依赖精确标定，这在量产中非常重要（标定会随时间漂移）。

**学习时间**：5-7天。

**推荐资源**：
- 论文：Bai et al., "TransFusion: Robust LiDAR-Camera Fusion for 3D Object Detection with Transformers", CVPR 2022
- 代码：`https://github.com/XuyangBai/TransFusion`

**检验标准**：能对比PointPainting、BEVFusion、TransFusion三种融合方法的融合策略差异；能在nuScenes上跑通至少一种多模态方法。

---

## Part 5: 3D语义分割与占据预测

### 5.1 3D语义分割

**学什么**：
- **Cylinder3D**：将笛卡尔坐标点云转换为柱坐标(r, theta, z)，在柱坐标下做3D卷积分割。优势是点云在柱坐标下分布更均匀，尤其是远距离区域。设计了残差风格的非对称3D卷积网络和置信度引导的refinement。
- **MinkUNet**：基于MinkowskiEngine的稀疏卷积UNet。在4D时空稀疏张量上做卷积，支持任意维度。架构为encoder-decoder + skip connection。是稀疏卷积语义分割的baseline。
- **SphereFormer**：在球形坐标下做Transformer注意力，设计radial window attention（沿径向方向做window attention），解决远处点稀疏导致的感受野不足问题。

**为什么学**：3D语义分割是占据预测的基础（占据预测可视为3D语义分割在体素空间的推广）。理解不同的3D特征提取方式（柱坐标、稀疏卷积、球形注意力）有助于选择和设计占据预测方案。

**学到什么程度**：能在MinkowskiEngine中跑通MinkUNet的训练；理解稀疏卷积语义分割的encoder-decoder设计。

**学习时间**：7-10天。

**推荐资源**：
- Cylinder3D论文：Zhu et al., "Cylindrical and Asymmetrical 3D Convolution Networks for LiDAR Segmentation", CVPR 2021
- Cylinder3D代码：`https://github.com/xinge008/Cylinder3D`
- MinkowskiEngine：`https://github.com/NVIDIA/MinkowskiEngine`
- MinkUNet：`https://github.com/mit-han-lab/spvnas`
- SphereFormer论文：Miao et al., "SphereFormer: Radial Window Transformer for Spherical Panorama", CVPR 2023
- 数据集：SemanticKITTI（`https://semantic-kitti.org/`）、nuScenes-lidarseg

---

### 5.2 Occupancy Prediction（占据预测）

**学什么**：
- **任务定义**：将自车周围的3D空间离散化为体素网格（如200x200x16），预测每个体素是否被占据（occupancy）以及其语义类别（如car、road、vegetation等）。与3D检测相比，占据预测能表示任意形状的物体（而非规则bbox），更适合不规则物体（如躺倒的人、碎片、施工区域）。
- **体素空间定义**：通常[-40m, 40m] x [-40m, 40m] x [-1m, 5.4m]，分辨率0.4m或0.5m。
- **GT生成**：用多帧LiDAR点云累积 + 射影投射(raycasting)获得稠密占据标注，或用mesh重建。

#### 5.2.1 SurroundOcc

**学什么**：
- 多相机输入 -> 图像backbone -> BEV特征（LSS或Transformer） -> 3D占据预测头。
- 使用多帧LiDAR累积生成GT占据标注。
- 评估指标：IoU和mIoU（对每个语义类别分别计算）。

**为什么学**：SurroundOcc提供了标准化的占据预测benchmark和GT生成流程。

**学习时间**：5-7天。

**推荐资源**：
- 论文：Wei et al., "SurroundOcc: A Multi-camera 3D Occupancy Prediction Benchmark for Autonomous Driving", 2023
- 代码：`https://github.com/weiyithu/SurroundOcc`

#### 5.2.2 OpenOccupancy

**学什么**：
- 提供更完整的占据标注（使用多帧累积 + 语义标注），包含800+场景的密集标注。
- 提供多模态占据预测baseline（纯视觉和LiDAR+Camera）。

**推荐资源**：
- 论文：Wang et al., "OpenOccupancy: A Large Scale Benchmark for Surround Semantic Occupancy Perception", ICCV 2023
- 代码：`https://github.com/JeffWang987/OpenOccupancy`

#### 5.2.3 Occ3D 和 FB-OCC

**学什么**：
- **Occ3D**：提供nuScenes上的占据标注benchmark，定义了统一的评估协议。
- **FB-OCC（Forward-Backward Occupancy）**：不仅预测当前帧的占据，还预测未来帧的占据（前向预测），支持运动规划。引入流场(flow field)建模体素的运动。

**推荐资源**：
- Occ3D代码：`https://github.com/Tsinghua-MARS-Lab/Occ3D`
- FB-OCC论文：Xiao et al., "FB-OCC: 3D Occupancy Prediction based on Forward-Backward View Transformation", 2023

#### 5.2.4 Tesla Occupancy Network

**学什么**：
- **离线知识蒸馏**：Tesla用多相机、多帧的离线3D重建pipeline生成高精度占据GT，然后将这个"teacher"的知识蒸馏到实时的"student"网络。
- **实时推理**：纯视觉输入，在车载芯片上实时运行（~10Hz）。
- **占据输出**：3D空间被划分为体素，每个体素预测是否被占据、语义类别、占据概率。
- **端到端集成**：占据预测结果直接供下游规划器使用，形成感知-规划闭环。

**为什么学**：Tesla是将占据预测真正量产落地的标杆。其技术博客和AI Day演讲详细介绍了整个系统设计。

**学到什么程度**：理解占据预测如何替代传统3D检测成为感知输出表示；理解知识蒸馏在占据预测中的应用。

**学习时间**：3-5天（主要理解设计思想）。

**推荐资源**：
- Tesla AI Day 2022 演讲（B站搜索"Tesla AI Day 2022"有中文字幕版）
- Tesla Occupancy Network技术博客：`https://www.notateslaapp.com/`
- 知乎："Tesla Occupancy Network详解"

**检验标准**：能解释占据预测与传统3D检测的区别和优势；能在SurroundOcc或Occ3D上跑通一个简单的占据预测baseline；理解为什么占据预测更适合不规则物体。

---

## Part 6: 开源工具链与数据集

### 6.1 MMDetection3D 完全指南

**学什么**：
- **安装**：PyTorch + mmcv-full/mmcv + mmdet + mmdet3d。推荐使用MIM安装（`pip install openmim; mim install mmengine mmcv mmdet mmdet3d`）。版本兼容性是关键：mmcv 1.x vs 2.x（mmcv 2.x对应mmengine，新版本mmdet3d已迁移）。
- **配置系统**：基于mmengine的config系统，使用`_base_`继承机制。配置文件层级：dataset config -> model config -> schedule config -> runtime config。
- **训练**：`python tools/train.py configs/pointpillars/hv_pointpillars_...`，支持单卡和多卡（dist_train.sh）。
- **测试/评估**：`python tools/test.py`，支持多种评估指标。
- **自定义数据集**：(1) 编写数据转换类（Dataset类）；(2) 编写data pipeline（LoadPointsFromFile, LoadAnnotations3D, RandomFlip3D等）；(3) 在config中注册。
- **自定义模型**：注册新backbone、head等组件，使用`@MODELS.register_module()`装饰器。
- **可视化**：使用`tools/misc/browse_dataset.py`可视化数据加载结果；使用`tools/analysis_tools/visualize_results.py`可视化检测结果。

**为什么学**：MMDetection3D是学术界和工业界使用最广泛的3D检测框架，大量论文基于它开发。掌握它是快速复现论文、开展实验的前提。

**学到什么程度**：能在MMDetection3D中训练PointPillars/CenterPoint/BEVFormer等主流模型；能自定义数据集适配自己的传感器配置；能修改config调参。

**学习时间**：14-21天（需要持续学习）。

**推荐资源**：
- 官方文档：`https://mmdetection3d.readthedocs.io/`
- GitHub：`https://github.com/open-mmlab/mmdetection3d`（star 5k+）
- 安装指南：`https://mmdetection3d.readthedocs.io/en/latest/get_started.html`
- 自定义数据集教程：`https://mmdetection3d.readthedocs.io/en/latest/advanced_guides/customize_dataset.html`
- B站：搜索"MMDetection3D使用教程"，有多个系列教程

**检验标准**：能从零开始在自定义数据集上训练一个PointPillars模型；能修改config切换不同的backbone/head。

---

### 6.2 OpenPCDet

**学什么**：
- OpenPCDet是另一个主流的LiDAR 3D检测代码库，由Shaoshuai Shi（PV-RCNN、PointRCNN作者）维护。
- **特点**：代码结构清晰、注释详细、支持PointRCNN/PV-RCNN/Part-A2/SECOND/PointPillars/CenterPoint等主流方法。
- **数据加载**：`DatasetTemplate`基类 -> `KittiDataset`/`NuScenesDataset`等。数据增强包括翻转、旋转、缩放、gt采样（copy-paste augmentation）。
- **模型构建**：基于registry机制，模块化设计。`build_detector()`根据config构建完整模型。

**为什么学**：OpenPCDet的代码比MMDetection3D更易读，适合入门学习。PV-RCNN、PointRCNN等经典方法的原始代码就在这里。

**学到什么程度**：能读懂OpenPCDet的核心代码流程（数据加载 -> 前向传播 -> loss计算 -> 后处理）。

**学习时间**：7-14天。

**推荐资源**：
- GitHub：`https://github.com/sshaoshuai/OpenPCDet`（star 4k+）
- README中有详细的安装和使用说明

---

### 6.3 数据集详解

#### 6.3.1 KITTI

**学什么**：
- **全称**：Karlsruhe Institute of Technology and Toyota Technological Institute数据集，2012年发布。
- **传感器**：1台Velodyne HDL-64E LiDAR、2台灰度相机+2台彩色相机（前向）、GPS/IMU。
- **任务**：2D/3D目标检测（7481训练+7518测试，Car/Pedestrian/Cyclist三类）、深度估计（投影视差图）、光流、视觉里程计、语义分割（SemanticKITTI扩展）、场景流（KITTI Scene Flow）。
- **数据划分**：3D检测标准划分：3712训练 / 3769验证 / 7518测试。train/val划分文件在官方开发工具包中。
- **标注格式**：3D标注包含(x, y, z, h, w, l, yaw, truncation, occlusion)。坐标系：相机坐标系（y朝下，z朝前）。
- **评估指标**：AP@IoU 0.7(Car)/0.5(Ped/Cyc)，分为Easy/Moderate/Hard三个难度级别（根据bbox大小、遮挡程度、截断程度划分）。

**为什么学**：KITTI是3D检测领域最经典的数据集，虽然规模较小但至今仍是重要benchmark。很多方法先在KITTI上验证再扩展到nuScenes。

**学到什么程度**：能加载并可视化KITTI的点云和2D/3D标注；理解Easy/Moderate/Hard的划分标准。

**推荐资源**：
- 官网：`https://www.cvlibs.net/datasets/kitti/`
- 开发工具包：`https://github.com/utiasSTARS/pykitti`
- SemanticKITTI：`https://semantic-kitti.org/`

#### 6.3.2 nuScenes

**学什么**：
- **规模**：1000个驾驶场景，每个场景20秒，1.4M帧图像，390K LiDAR扫描。
- **传感器**：6台环绕相机、1台32线LiDAR、5台毫米波雷达、GPS/IMU。
- **标注**：23类物体，1.1M 3D bounding box标注，包含物体属性（可见性、活动状态等）。
- **特点**：(1) 全360度感知（6相机环绕）；(2) 多模态（camera+LiDAR+radar）；(3) 完整的标注属性（包括速度、活动状态）；(4) 支持tracking标注（nuScenes-tracking）。
- **评估指标**：NDS（nuScenes Detection Score）= mAP + 5个TP指标的平均（mATE位置误差、mASE尺度误差、mAOE朝向误差、mAVE速度误差、mAAE属性误差）。
- **数据划分**：700训练 / 150验证 / 150测试。
- **开发工具包**：Python API，支持按场景/帧/类别查询，内置可视化。

**为什么学**：nuScenes是BEV感知和多模态融合方向最重要的数据集。几乎所有BEV论文（BEVDet, BEVFormer, BEVFusion等）都在nuScenes上评估。

**学到什么程度**：能熟练使用nuscenes-devkit加载数据、可视化多模态数据、导出自定义格式的数据；理解NDS评估指标。

**推荐资源**：
- 官网：`https://www.nuscenes.org/`
- 开发工具包：`https://github.com/nutonomy/nuscenes-devkit`
- 官方教程notebook：`https://github.com/nutonomy/nuscenes-devkit/tree/master/python-sdk/tutorials`
- 自定义数据导出：搜索"nuScenes数据集自定义导出"

#### 6.3.3 Waymo Open Dataset

**学什么**：
- **规模**：最大规模的自动驾驶公开数据集之一。1150段场景（约230K帧），包含城市、郊区、高速公路等多种场景。
- **传感器**：5台LiDAR（1台中距HDL-64E + 4台短距HDL-32E）、5台环绕相机。
- **标注**：4类（Vehicle, Pedestrian, Cyclist, Sign），2D和3D标注，tracking标注。
- **特点**：(1) 数据量大，适合大数据训练；(2) 标注质量高；(3) 有专门的运动预测子集（WOD Motion Prediction）；(4) TFRecord格式。
- **评估指标**：mAP@L1（难度级别1，IoU 0.7/0.5/0.5）和mAP@L2（难度级别2，IoU 0.7/0.5/0.5，额外排除超过一定距离的物体）。

**为什么学**：规模最大，训练效果最好。很多方法在Waymo上训练后泛化性更好。

**学到什么程度**：能加载和处理Waymo TFRecord格式数据；了解WOD的评估方式。

**推荐资源**：
- 官网：`https://waymo.com/open/`
- 开发工具包：`https://github.com/waymo-research/waymo-open-dataset`
- 评估工具：`https://github.com/waymo-research/waymo-open-dataset/tree/master/src/waymo_open_dataset/metrics`

#### 6.3.4 Argoverse

**学什么**：
- **Argoverse 1**：包含3D tracking（290K帧，LiDAR+环绕相机）和Motion Forecasting（324K场景序列）两个子集。
- **Argoverse 2**：大幅扩展，1000个场景，支持3D检测、tracking、运动预测、stereo深度、点云分割等多项任务。
- **特点**：(1) 包含HD Map（高精地图），包含车道线、人行道、交通区域等语义信息；(2) 长序列数据，适合时序任务和预测任务。

**为什么学**：Argoverse的运动预测子集是预测任务的标准benchmark；其HD Map信息对研究map-based感知和规划非常重要。

**推荐资源**：
- 官网：`https://www.argoverse.org/`
- 开发工具包：`https://github.com/argoverse/argoverse-api`

#### 6.3.5 自建海洋数据集策略

**学什么**：
- **传感器选型**：海洋环境特殊，需要考虑防水、防腐蚀。LiDAR推荐Livox（固态、体积小好密封）或Hesai（工业级防水壳体）；相机推荐工业防水相机（如FLIR防水版）。
- **数据采集**：选择典型场景（码头、航道、开阔水域）和典型目标（船舶、浮标、码头结构、漂浮物）；注意不同天气/光照条件；GPS+IMU提供pose标注。
- **标注策略**：(1) 3D bbox标注：使用LabelCloud或自建标注工具，标注船舶、浮标等；(2) 语义分割标注：海洋、天空、船舶、码头等类别；(3) 考虑半自动标注：先用预训练模型生成初始标注，再人工修正。
- **数据格式转换**：将自采数据转换为KITTI或nuScenes格式，复用现有代码库。
- **数据增强**：海洋场景数据少，需要更多增强：GT-AUG（从其他场景copy-paste目标到当前场景）、天气模拟增强。
- **小样本策略**：预训练->微调（用KITTI/nuScenes预训练backbone，在海洋数据集上fine-tune）。

**为什么学**：自建数据集是研究生阶段几乎必做的工作。理解数据集构建流程（采集->标注->划分->评估）对开展真实研究至关重要。

**学到什么程度**：能制定一份完整的数据采集和标注方案；能将自建数据集转换为标准格式并在MMDetection3D/OpenPCDet中训练。

**学习时间**：持续性工作。

**推荐资源**：
- 标注工具：`https://github.com/ch-sa/labelCloud`（3D bbox标注）
- 数据转换参考：`https://github.com/poodarchu/Det3D`（支持多种数据集格式转换）
- 论文参考：搜索"marine object detection"、"maritime LiDAR"相关论文
- nuScenes数据格式规范：作为自建数据集格式设计的参考

**检验标准**：能制定一份包含传感器选型、数据采集方案、标注规范、数据格式设计的完整数据集构建计划书。

---

## 附录：建议的学习路径和时间规划

### 第一阶段（1-2个月）：基础能力

1. Python + PyTorch 基础（如已有可跳过）
2. 点云数据格式学习 + Open3D/PCL入门
3. 点云体素化原理
4. PointNet + PointNet++ 论文精读 + 代码复现

### 第二阶段（2-3个月）：3D检测入门

1. SECOND + PointPillars（精学代码实现）
2. CenterPoint
3. MMDetection3D 工具链熟悉
4. 在KITTI上训练PointPillars并调优

### 第三阶段（3-4个月）：BEV感知

1. LSS 论文精读 + 代码理解
2. BEVDet/BEVDepth
3. BEVFormer（理解Transformer-based BEV构建）
4. 多模态融合（BEVFusion、TransFusion）
5. 在nuScenes上训练至少一种BEV方法

### 第四阶段（4-6个月）：前沿方向

1. 3D语义分割（MinkUNet）
2. 占据预测（SurroundOcc/Occ3D）
3. 端到端自动驾驶（UniAD/VAD，了解即可）
4. 确定研究生课题方向，开始深入研究

### 持续事项

- 每周阅读1-2篇最新的3D感知论文（关注CVPR/ICCV/ECCV/NeurIPS/ICRA/RAL/TPAMI）
- 关注nuScenes和Waymo排行榜的最新方法
- 建立自己的论文笔记库（推荐Notion或Obsidian）
- 多写代码，多跑实验，多看源码

---

以上指南涵盖了自动驾驶3D感知与BEV感知方向从基础到前沿的完整知识体系。建议根据个人基础灵活调整每个阶段的时间分配，但"动手跑代码"比"只看论文"重要得多。每学一个方法，务必在对应数据集上跑通训练和测试，这是工程落地型学生的核心竞争力。

---

# 模块五：路径规划与决策控制

## 自动驾驶方向 - 路径规划与决策控制 学习指南

> 适用对象：机器人工程本科背景，已具备自动控制原理与现代控制理论基础，即将攻读自动驾驶方向硕士研究生。全文以工程落地为导向，控制在7000字以内。

---

## Part 1: 全局路径规划

全局路径规划解决"从A到B怎么走"的问题，是在已知地图上搜索一条无碰撞的最优（或可行）路径。这是所有后续规划与控制的起点。

### 1.1 Dijkstra算法

**学什么**：基于图搜索的最短路径算法。理解优先队列、松弛操作、代价函数设计。重点掌握栅格地图建模（占据栅格、八邻域/四邻域）、代价值设定（距离代价、障碍物膨胀）。

**为什么学**：Dijkstra是所有基于图搜索规划器的思想原型。A*、D*等都是它的变体。理解Dijkstra才能理解启发式搜索为何有效。

**学到什么程度**：能在C++/Python中从零实现Dijkstra在栅格地图上的搜索，理解时间复杂度O(V^2)和使用优先队列优化至O((V+E)logV)的原理。

**学习时间**：2-3天

**推荐资源**：
- 书籍：《算法导论》第24章"单源最短路径"
- 视频：MIT 6.006 Introduction to Algorithms - Dijkstra讲座（YouTube/B站搬运）
- 博客：CSDN搜索"Dijkstra栅格路径规划"，配合matplotlib可视化理解

**检验标准**：能在200x200栅格地图上从零手写Dijkstra并输出路径，能解释为何Dijkstra不适合大规模实时规划。

### 1.2 A*算法

**学什么**：在Dijkstra基础上引入启发函数h(n)（欧几里得距离、曼哈顿距离、切比雪夫距离），理解f(n)=g(n)+h(n)的设计，理解可采纳性（admissible）和一致性（consistent）条件。

**为什么学**：A*是自动驾驶中全局规划最常用的搜索算法，Apollo的Routing模块就基于改进A*。理解启发式设计直接决定搜索效率。

**学到什么程度**：手写A*，能对比不同启发函数的搜索效率，理解权重A*（Weighted A*）的权衡。

**学习时间**：2天

**推荐资源**：
- 论文：Hart, P.E. et al. "A Formal Basis for the Heuristic Determination of Minimum Cost Paths" (1968)
- GitHub：`zhm-real/PathPlanning` (搜索A*模块，含可视化)
- B站：搜索"A*算法 路径规划 可视化"，推荐Up主"古月居"的相关视频
- 实践：Python实现A*并用matplotlib动画展示搜索过程

**检验标准**：能快速手写A*，能解释A*在最坏情况下退化为Dijkstra的原因，能在ROS2中调用nav2的A* planner。

### 1.3 JPS（Jump Point Search）

**学什么**：A*在均匀代价栅格地图上的加速变体。核心思想是利用对称性剪枝，识别"跳点"（Jump Point），跳过大量对称路径。

**为什么学**：在大规模均匀栅格地图上，JPS比A*快一个数量级。适合开阔场景的全局规划加速。

**学到什么程度**：理解跳点识别规则（强迫邻居、直线跳跃、对角跳跃），能阅读JPS源码并理解其剪枝逻辑。不需要从零手写，但要能修改和调试。

**学习时间**：2天

**推荐资源**：
- 论文：Harabor, D. & Grastien, A. "Online Graph Pruning for Pathfinding on Grid Maps" (2011)
- GitHub：`zhehaoli/jump-point-search` 或搜索 "JPS path planning"
- 博客：知乎搜索"JPS算法详解"

**检验标准**：能解释JPS为何在均匀代价地图上优于A*，能说清在非均匀代价（如道路有不同限速区域）时JPS失效的原因。

### 1.4 D* Lite

**学什么**：增量式启发式搜索算法。当环境动态变化（发现新障碍物、地图更新）时，不需要重新全图搜索，而是在已有搜索树上增量更新。

**为什么学**：自动驾驶中环境是动态的，全局路径可能需要频繁局部修正。D* Lite是理解增量规划思想的基础，也是移动机器人领域经典方法。

**学到什么程度**：理解"右状态"（rhs值）和g值的双值机制，理解增量更新的核心逻辑。能阅读现有D* Lite实现代码，理解其与A*的联系。

**学习时间**：3天

**推荐资源**：
- 论文：Koenig, S. & Likhachev, M. "D* Lite" (AAAI 2002)
- GitHub：搜索"D star lite python implementation"
- 书籍：《Planning Algorithms》(Steven LaValle) 第5章，免费在线版 http://planning.cs.uiuc.edu/

**检验标准**：能用流程图描述D* Lite的增量更新过程，能解释其在动态环境中的计算优势。

### 1.5 RRT（快速随机探索树）

**学什么**：基于采样的单查询规划算法。核心操作：随机采样、最近邻搜索（KD-Tree）、步长扩展、碰撞检测。理解RRT的概率完备性（probabilistically complete）。

**为什么学**：在高维空间（如考虑车辆朝向的SE(2)空间）和复杂约束下，图搜索方法难以扩展，基于采样的方法是主流方案。RRT是后续RRT*、Informed RRT*、Kinodynamic RRT*的基础。

**学到什么程度**：Python/C++手写2D RRT，理解并使用nanoflann等KD-Tree库做最近邻查询，理解RRT路径不最优的根本原因。

**学习时间**：3天

**推荐资源**：
- 论文：LaValle, S.M. "Rapidly-Exploring Random Trees" (1998)
- GitHub：`zhm-real/PathPlanning` 的RRT模块；`OMPL`库（Open Motion Planning Library）
- 视频：B站搜索"RRT算法 动画讲解"
- 书籍：《Planning Algorithms》第5.5节

**检验标准**：能手写RRT，能可视化搜索树生长过程，能清楚说明RRT的"偏向目标"策略（goal bias）的作用。

### 1.6 RRT*与Informed RRT*

**学什么**：RRT*在RRT基础上加入rewiring（重布线）操作，使路径渐近最优。Informed RRT*在找到初始可行解后，将采样空间从整个C-space收缩到以起点和终点为焦点的椭球内，加速收敛。

**为什么学**：工程中需要的路径不仅可行，还要平滑、安全。RRT*和Informed RRT*是目前工业界常用的采样规划算法。

**学到什么程度**：手写RRT*（含rewiring），理解Informed RRT*的椭球采样几何原理。能用OMPL库调用相关planner并配置参数。

**学习时间**：3-4天

**推荐资源**：
- 论文：Karaman, S. & Frazzoli, E. "Sampling-based Algorithms for Optimal Motion Planning" (2011)；Gammell, J.D. et al. "Informed RRT*" (2014)
- GitHub：`OMPL`官方仓库 https://github.com/ompl/ompl
- B站：搜索"RRT*算法讲解"

**检验标准**：能对比RRT和RRT*生成路径的质量差异（路径代价、平滑度），能解释rewiring的数学直觉（三角不等式）。

### 1.7 PRM（概率路线图）

**学什么**：多查询规划方法。分为学习阶段（随机撒点+碰撞检测+连接邻居构建路线图）和查询阶段（在路线上图上做图搜索）。理解与RRT的适用场景差异。

**为什么学**：PRM适合固定环境下多次查询的场景（如仓库机器人）。在自动驾驶中用于离线构建道路拓扑的参考思路。

**学到什么程度**：理解PRM两阶段流程，能用OMPL实现PRM，能与RRT对比（多查询vs单查询、离线vs在线）。

**学习时间**：1-2天

**推荐资源**：
- 论文：Kavraki, L. et al. "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces" (1996)
- GitHub：OMPL库自带PRM实现
- 书籍：《Probabilistic Robotics》(Thrun, Burgard, Fox) 第14章

**检验标准**：能说明PRM和RRT各自适用场景，能在OMPL中配置PRM参数并可视化路线图。

---

## Part 2: 局部路径规划

局部路径规划解决"在全局路径引导下，如何实时避障并生成可执行轨迹"的问题。

### 2.1 DWA（Dynamic Window Approach）动态窗口法

**学什么**：基于速度空间采样的局部规划方法。核心步骤：(1)在(v, omega)速度空间中根据动力学约束确定动态窗口；(2)采样速度对；(3)前向模拟轨迹；(4)对轨迹打分（目标方向、障碍物距离、速度）；(5)选最优速度。

**为什么学**：DWA是ROS/ROS2 Navigation中经典的局部规划器，直接输出速度指令(cmd_vel)，与底层控制无缝对接。理解DWA才能理解局部规划的本质——在约束空间中实时搜索。

**学到什么程度**：Python手写DWA，理解航向评价、障碍物评价、速度评价的权重设计。能在ROS2中配置和调试DWA参数。

**学习时间**：3天

**推荐资源**：
- 论文：Fox, D. et al. "The Dynamic Window Approach to Collision Avoidance" (1997)
- GitHub：`zhm-real/PathPlanning` 的DWA模块；ROS2 Navigation2的`dwb_local_planner`
- B站：古月居"ROS Navigation系列"视频
- 博客：知乎"DWA动态窗口法详解"

**检验标准**：能手写DWA并在2D仿真中运行，能调参并解释各项权重对行为的影响（过于保守vs激进）。

### 2.2 TEB（Timed Elastic Band）规划器

**学什么**：将路径表示为一系列带时间信息的位姿点（elastic band），通过图优化（g2o框架）同时优化路径形状和时间分配。核心概念：弹性带、时间参数化、超图优化、多目标约束（障碍物距离、速度/加速度限制、非完整性约束）。

**为什么学**：TEB是ROS Navigation中处理差速/阿克曼车辆最有效的局部规划器之一，能生成时间最优且满足车辆约束的轨迹。在自动驾驶和机器人导航中广泛使用。

**学到什么程度**：理解TEB的优化框架（顶点=位姿，边=约束），能在ROS2中配置TEB参数，理解各约束边的物理意义。不需要手写底层优化器，但要能分析优化失败的原因。

**学习时间**：4-5天

**推荐资源**：
- 论文：Rösmann, C. et al. "Trajectory Modification Considering Dynamic Constraints of Autonomous Robots" (ROBOTIK 2012)；后续论文"Online Trajectory Optimization" (2013, 2015)
- GitHub：`rst-tu-dortmund/teb_local_planner`（ROS版本）；`ROS2的nav2_regulated_pure_pursuit`可对比学习
- 博客：CSDN搜索"TEB规划器原理详解"

**检验标准**：能在ROS2仿真中配置TEB规划器处理阿克曼车辆导航，能解释"elastic band"的物理直觉和时间参数化的作用。

### 2.3 人工势场法（APF, Artificial Potential Field）

**学什么**：目标产生引力场，障碍物产生斥力场，机器人在合力方向运动。理解引力函数、斥力函数设计，理解局部极小值问题及其逃逸策略（随机扰动、沿等势面切线运动等）。

**为什么学**：APF是最直观的反应式避障方法，思想影响了大量后续算法（如Velocity Obstacle系列）。在简单场景中快速有效，且易于与速度控制结合。

**学到什么程度**：Python手写APF，能可视化势场分布和机器人轨迹，理解并实现至少一种局部极小值逃逸策略。

**学习时间**：1-2天

**推荐资源**：
- 论文：Khatib, O. "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots" (1986)
- GitHub：`zhm-real/PathPlanning` 的APF模块
- 书籍：《机器人学导论》(Craig) 或《机器人学：建模、规划与控制》(Siciliano)

**检验标准**：能手写APF并在2D仿真中观察局部极小值现象，能实现逃逸策略并对比效果。

---

## Part 3: 运动规划（自动驾驶核心）

这是自动驾驶规划的核心技术栈，将全局和局部规划思想统一到连续轨迹生成框架中。

### 3.1 Frenet坐标系

**学什么**：将笛卡尔坐标(x,y)转换为沿参考线的纵向位移s和横向偏移d的坐标系。理解Frenet框架下的运动分解：纵向运动（沿道路方向）和横向运动（偏离道路方向）解耦。理解参考线的选取（通常为全局路径或车道中心线）、曲率计算、坐标转换的数学推导。

**为什么学**：自动驾驶中几乎所有规划算法（Lattice、MPC、QP优化）都在Frenet坐标系下工作。它天然将问题分解为"沿道路走多快"和"离道路中心多远"两个子问题，大幅简化规划。

**学到什么程度**：能推导Frenet变换的完整数学公式，能在代码中实现(x,y,theta)到(s,d,d')的转换和反变换。理解曲率在Frenet中的表示。

**学习时间**：3-4天

**推荐资源**：
- 论文：Werling, M. et al. "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame" (ICRA 2010) -- 这是经典必读论文
- 视频：B站搜索"Frenet坐标系 自动驾驶"，推荐Up主"王方浩看世界"或"自动驾驶Studio"
- GitHub：`AtsushiSakai/PythonRobotics` 的Frenet Frame模块
- 博客：知乎"自动驾驶中的Frenet坐标系详解"

**检验标准**：能从零实现Frenet坐标变换，能在给定参考线和一组(x,y)点时正确计算(s,d)。

### 3.2 多项式轨迹与贝塞尔曲线

**学什么**：
- **五次多项式（Quintic Polynomial）**：给定起点和终点的(位置, 速度, 加速度)共6个边界条件，唯一确定五次多项式。用于纵向和横向轨迹生成。
- **四次多项式（Quartic Polynomial）**：纵向运动中终点时间自由时使用。
- **贝塞尔曲线**：由控制点定义的参数曲线，理解de Casteljau算法、凸包性质、端点插值性质。
- **B样条**：局部控制特性，理解节点向量、基函数、阶数的概念。

**为什么学**：多项式是Lattice Planner的轨迹生成核心；贝塞尔/B样条在路径平滑、车道线拟合、parking规划中广泛使用。这些是自动驾驶轨迹生成的"基本积木"。

**学到什么程度**：能手推五次多项式系数（6个方程6个未知数），能用Python实现贝塞尔曲线生成和控制点调节。理解B样条的局部修改性质。不需要手写B样条求解器，但要能调用库并理解参数含义。

**学习时间**：4-5天

**推荐资源**：
- 论文：Werling 2010 (同上)
- GitHub：`AtsushiSakai/PythonRobotics` 的Quintic Polynomial模块；`Pomax/bezierinfo` (贝塞尔曲线可视化交互教程)
- 书籍：《计算机图形学》(Foley) 中的曲线章节
- B站：搜索"贝塞尔曲线 自动驾驶"、"五次多项式轨迹规划"

**检验标准**：给定6个边界条件，能手算五次多项式系数并用Python画出轨迹；能用控制点调节贝塞尔曲线形状。

### 3.3 Lattice Planner（格栅规划器）

**学什么**：在Frenet坐标系下，沿参考线在纵向和横向上离散采样终点状态，用多项式连接起点和各终点候选，生成多条候选轨迹，通过代价函数（平滑性、安全性、效率、舒适性）筛选最优轨迹。理解横向和纵向的独立采样与组合、轨迹评估与选择、实时性保障。

**为什么学**：Lattice Planner是目前自动驾驶量产方案中最主流的规划框架之一（Apollo、百度、小鹏等均采用类似架构）。它将运动规划问题优雅地转化为有限候选轨迹的评估问题。

**学到什么程度**：能用Python实现完整的Lattice Planner框架（Frenet坐标系 + 五次多项式 + 代价函数 + 轨迹筛选）。理解与ROS2 Nav2的Lattice Planner插件的区别。能在仿真中生成可跟踪的轨迹。

**学习时间**：5-7天

**推荐资源**：
- 论文：Werling 2010 (核心论文)；McNaughton, M. "Motion Planning for Autonomous Driving with a Conformal Spatiotemporal Lattice" (2011)
- GitHub：`AtsushiSakai/PythonRobotics` 的Lattice Planner模块；Apollo开源代码 `apollo/modules/planning/lattice/`
- 书籍：《自动驾驶汽车规划技术》(高翔/王乃岩 等)
- B站：搜索"Lattice Planner 自动驾驶规划"

**检验标准**：能运行并理解Lattice Planner的完整流程，能在仿真中生成多条候选轨迹并可视化筛选结果。

### 3.4 基于优化的轨迹规划（QP/OSQP）

**学什么**：将轨迹规划建模为带约束的二次规划（QP）问题。目标函数通常包含：路径平滑性（最小化高阶导数）、与参考线的偏差、与障碍物的距离。约束包括：边界约束、障碍物避让约束（用凸空间/半平面表示）、速度/加速度限制。理解OSQP（Operator Splitting QP）求解器的接口和使用。

**为什么学**：工业级规划器（Apollo EM Planner、Lattice+QP混合架构）都在多项式采样之后用QP做轨迹平滑和修正。QP是将"粗糙轨迹"变为"可执行轨迹"的标准工具。

**学到什么程度**：能将轨迹平滑问题建模为标准QP形式（min x'Px + q'x, s.t. l <= Ax <= u），能用Python调用OSQP/CVXPY求解。理解离散化、线性化的必要性。

**学习时间**：4-5天

**推荐资源**：
- OSQP官方文档：https://osqp.org/ （有Python/MATLAB教程）
- GitHub：`osqp/osqp`；`cvxpy/cvxpy`（建模更友好）
- Apollo源码：`apollo/modules/planning/math/smoothing_spline/`
- 书籍：《Convex Optimization》(Boyd) -- 只需读前4章理解QP即可，免费PDF：https://web.stanford.edu/~boyd/cvxbook/
- B站：搜索"二次规划 自动驾驶 轨迹优化"

**检验标准**：能将路径平滑问题表述为QP并用OSQP求解，能可视化优化前后的轨迹对比。

---

## Part 4: MPC模型预测控制（学生优势方向）

这是你的核心竞争力方向。扎实的控制理论基础让你能深入理解MPC的数学本质，而非仅调包。

### 4.1 车辆运动学与动力学模型

**学什么**：
- **运动学模型（Kinematic Bicycle Model）**：以车辆后轴中心为参考点，状态[x, y, theta, v]，控制量[加速度a, 前轮转角delta]。推导微分方程，理解小角度假设下的线性化。
- **动力学模型（Dynamic Bicycle Model）**：引入轮胎侧偏力（线性轮胎模型Pacejka简化版），状态增加侧向速度vy和横摆角速度r。理解前后轮侧偏角、侧偏刚度的物理意义。
- **离散化**：前向欧拉法将连续模型离散化为差分方程，这是MPC求解的前提。

**为什么学**：MPC的精度直接取决于车辆模型的准确度。运动学模型适合低速（<5m/s），动力学模型适合高速。这是你将控制理论（状态空间建模）直接应用于自动驾驶的最佳切入点。

**学到什么程度**：能手推运动学Bicycle Model的微分方程，能用Python实现连续和离散模型，能做简单的开环仿真验证模型正确性。能理解动力学模型中侧偏力的物理意义。

**学习时间**：3-4天

**推荐资源**：
- 书籍：《Vehicle Dynamics and Control》(Rajamani) -- 自动驾驶车辆动力学经典教材
- 课程：Coursera "Self-Driving Cars Specialization" by University of Toronto, Course 2 "State Estimation and Localization"
- GitHub：`AtsushiSakai/PythonRobotics` 的Bicycle Model模块
- 博客：知乎"车辆运动学模型和动力学模型详解"

**检验标准**：能从牛顿力学/运动学约束出发推导Bicycle Model，能在代码中验证模型（给定恒定控制输入，观察轨迹是否符合物理直觉）。

### 4.2 MPC跟踪控制器

**学什么**：MPC的核心思想——在每个时间步求解一个有限时域最优控制问题：min Σ(跟踪误差^2 + 控制量^2 + 控制增量^2)，s.t. 车辆模型约束、状态约束、控制约束。理解预测时域N、控制时域M的选取，理解终端代价和终端约束的作用。

**关键数学**：将非线性MPC问题线性化为QP（Sequential Quadratic Programming思路），或直接使用非线性求解器。理解状态增广（将参考轨迹纳入状态）。

**为什么学**：MPC是自动驾驶横向/纵向控制的工业标准方案。它能系统地处理多约束（车道边界、速度限制、转向角限制）和多目标（跟踪精度、舒适性、效率），这是PID/LQR做不到的。

**学到什么程度**：能用Python + CasADi（或cvxpy+OSQP）实现完整的MPC轨迹跟踪控制器，包含运动学模型、参考轨迹导入、约束设置、求解和结果可视化。

**学习时间**：5-7天

**推荐资源**：
- 书籍：《Model Predictive Control: Theory, Computation, and Design》(Rawlings, Mayne, Morari) -- 经典教材，读前5章
- 课程：ETH Zürich "Model Predictive Control"课程（有课件和练习，B站有搬运）
- GitHub：`AtsushiSakai/PythonRobotics` 的MPC模块；`casadi/casadi` 官方示例
- 实践教程：`MPC-Bearings` 或搜索 "MPC trajectory tracking tutorial Python"
- B站：搜索"MPC模型预测控制 自动驾驶"，推荐"控制相关"和"DR_CAN"频道

**检验标准**：能独立实现MPC跟踪控制器，在有噪声和初始偏差的情况下验证跟踪性能，能分析预测时域、权重矩阵对控制效果的影响。

### 4.3 MPC vs 纯追踪 vs Stanley

**学什么**：
- **纯追踪（Pure Pursuit）**：几何方法，计算到达前方look-ahead点所需的曲率，输出转向角。核心参数：前瞻距离ld（与速度成正比）。
- **Stanley控制器**：结合横向误差的几何反馈和航向误差反馈，前端点控制。
- **MPC**：与上述两种方法在理论基础、计算复杂度、约束处理能力上的全面对比。

**为什么学**：纯追踪和Stanley是工程中最简单的横向控制器，作为baseline和MPC形成对照。量产项目中往往低速用纯追踪，高速用MPC。理解三者差异是做方案选型的基础。

**学到什么程度**：三种控制器均手写实现，在相同参考轨迹和初始偏差下对比跟踪效果。能系统总结三者优缺点表格。

**学习时间**：3-4天

**推荐资源**：
- 论文：Coulter, R. "Implementation of the Pure Pursuit Path Tracking Algorithm" (CMU, 1992)；Hoffmann, G. et al. "Autonomous Automobile Trajectory Tracking for Off-Road Driving" (Stanley控制器, 2007)
- GitHub：`AtsushiSakai/PythonRobotics` 含Pure Pursuit和Stanley实现
- B站：搜索"Pure Pursuit 自动驾驶"、"Stanley控制器"

**检验标准**：能在同一场景下运行三种控制器并输出对比图，能写出总结性的优缺点对比表（计算量、约束处理、高速性能、实现复杂度）。

### 4.4 CasADi/OSQP工程实现

**学什么**：
- **CasADi**：非线性优化建模框架，支持自动微分，能构建NLP问题并调用IPOPT等求解器。重点学习：符号变量定义、函数构建、NLP问题构建和求解。
- **OSQP**：针对凸QP的高效求解器。重点学习：P、q、A、l、u矩阵构建，warm-start利用，求解器参数调优。
- **工程技巧**：实时性保障（限制N值、warm-start、热启动）、数值稳定性（正则化）、调参方法论。

**为什么学**：CasADi + IPOPT用于非线性MPC（NMPC），OSQP用于线性化MPC/凸QP问题。这是工程落地的必备工具链。

**学到什么程度**：能用CasADi搭建NMPC控制器并求解，能用OSQP求解线性化的MPC问题，理解两种方案的适用场景和速度差异。

**学习时间**：4-5天

**推荐资源**：
- CasADi官方文档：https://web.casadi.org/ （有丰富的Python示例）
- OSQP官方文档：https://osqp.org/
- GitHub：`casadi/casadi` 的examples/python目录；`osqp/osqp` 的examples
- B站：搜索"CasADi MPC 自动驾驶"

**检验标准**：能用CasADi实现一个完整的NMPC控制问题并求解，能用OSQP实现线性MPC并对比求解时间。

---

## Part 5: 行为决策

行为决策解决"现在该做什么"的问题（换道、跟车、超车、让行、停车等）。

### 5.1 有限状态机（FSM）与行为树（Behavior Tree）

**学什么**：
- **FSM**：状态、转移条件、事件驱动。设计自动驾驶状态（巡航、跟车、换道、紧急制动等）和转移逻辑。
- **行为树**：选择节点(Selector)、序列节点(Sequence)、条件节点(Condition)、动作节点(Action)。理解与FSM的对比（可扩展性、可读性、模块化）。
- **决策树**：基于特征的分类/回归，理解信息增益、基尼系数等分裂准则（作为机器学习入门）。

**为什么学**：FSM和行为树是自动驾驶行为决策的两大主流工程实现方式。Apollo使用FSM，ROS2 Nav2使用行为树。工程落地必须掌握至少一种。

**学到什么程度**：能用Python/C++实现一个包含5-8个状态的自动驾驶FSM，能在ROS2中编写自定义行为树节点并组装行为树。

**学习时间**：3-4天

**推荐资源**：
- GitHub：`BehaviorTree/BehaviorTree.CPP`（ROS2 Nav2使用的BT库）；`ros-planning/geometry2`中的行为树教程
- 书籍：《Behavior Trees in Robotics and AI》(Michele Colledanchise, 免费PDF)
- Apollo源码：`apollo/modules/planning/tasks/` 中的行为决策模块
- B站：搜索"行为树 ROS2 自动驾驶"

**检验标准**：能在ROS2中用行为树框架实现简单的自动驾驶场景（如：巡航中检测到前车则切换跟车，出现空档则切换超车），能清晰画出行为树结构图。

### 5.2 强化学习决策入门

**学什么**：MDP基础（状态、动作、奖励、转移概率）、Q-Learning、DQN（Deep Q-Network）。理解离线策略(off-policy)和在线策略(on-policy)、经验回放、目标网络。了解Policy Gradient（REINFORCE）和PPO的基本思想。

**为什么学**：强化学习在自动驾驶决策中的应用（如高速换道决策、交互式博弈）是研究热点。作为硕士生，理解RL基础能让你阅读相关论文并尝试实验。

**学到什么程度**：实现一个简单的DQN解决Gym环境中的CartPole或MountainCar问题。理解RL用于自动驾驶的局限性（安全性保证、可解释性差）。不深入高级RL算法，重点是建立基本认知。

**学习时间**：5-7天

**推荐资源**：
- 书籍：《Reinforcement Learning: An Introduction》(Sutton & Barto) 第1-6章，免费在线版：http://incompleteideas.net/book/the-book-2nd.html
- 课程：David Silver RL课程（B站有搬运）；Spinning Up in Deep RL (OpenAI) https://spinningup.openai.com/
- GitHub：`openai/gym`；`DQN`经典实现参考 `dennybritz/reinforcement-learning`
- B站：搜索"DQN 强化学习入门"

**检验标准**：能用PyTorch实现DQN并训练Gym环境，能解释为什么RL在真实自动驾驶中难以直接使用（安全性、sim-to-real gap、奖励设计困难）。

---

## Part 6: Nav2导航框架

### 6.1 Nav2架构详解与自定义插件

**学什么**：ROS2 Navigation2的整体架构——行为服务器(BT Navigator)、规划服务器(Planner Server)、控制器服务器(Controller Server)、恢复服务器(Recovery Server)。理解生命周期节点(Lifecycle Node)、动作服务器(Action Server)通信机制。核心是插件化架构：所有算法都通过pluginlib动态加载，可以替换或自定义。

**为什么学**：Nav2是目前开源机器人导航的事实标准框架。自动驾驶原型系统或园区无人车项目中，Nav2是最快速的集成方案。理解其架构能让你高效集成自己的规划和控制算法。

**学到什么程度**：(1)能在Nav2中替换默认的全局/局部规划器为自定义插件；(2)能编写自定义Controller插件（如实现MPC控制器插件）；(3)能修改行为树XML定义导航逻辑。理解Costmap2D的分层机制。

**学习时间**：5-7天

**推荐资源**：
- 官方文档：https://docs.nav2.org/ （必读，非常详细）
- GitHub：`ros-navigation/navigation2` 官方仓库，重点看`nav2_controller`、`nav2_planner`、`nav2_bt_navigator`包
- 教程：官方教程 Writing a New Controller Plugin / Writing a New Planner Plugin
- B站：搜索"Nav2 自定义插件"、"ROS2 Navigation2 入门到精通"，推荐"古月居"和"鱼香ROS"系列
- 书籍：《ROS2 Navigation Tuning Guide》(GitHub上有免费版)

**检验标准**：能独立编写一个Nav2 Controller插件（例如将Part 4中实现的MPC封装为Nav2插件）并在仿真中运行，能修改行为树实现自定义导航逻辑。

---

## Part 7: 控制理论在自动驾驶中的应用（你的加分项）

充分利用你的自动控制原理和现代控制理论背景，将已有知识直接对接到自动驾驶控制问题。

### 7.1 PID控制

**学什么**：PID在自动驾驶中的应用——纵向速度控制（油门/制动）和横向转向控制。理解前馈+反馈结构在车辆控制中的重要性（前馈处理已知的曲率/期望加速度，反馈处理未知扰动）。理解抗积分饱和(Anti-Windup)、微分滤波等工程技巧。

**为什么学**：PID是量产车辆底层控制的标配（即便上层用MPC，底层仍可能用PID做执行器控制）。你的控制理论基础使你能超越"调参黑魔法"，用根轨迹/频域分析指导PID参数整定。

**学到什么程度**：能设计前馈+PID反馈结构做速度和转向控制，能用频域分析方法（Bode图、稳定裕度）分析和整定参数。

**学习时间**：2天（你已有基础，重点在应用迁移）

**推荐资源**：
- 你的考研教材（如胡寿松《自动控制原理》）的PID章节
- 实践：在Carla或LGSVL仿真器中实现PID纵向控制

**检验标准**：能在仿真中实现PID速度跟踪和横向控制，能画出Bode图分析闭环性能。

### 7.2 LQR最优控制

**学什么**：LQR在自动驾驶横向控制中的应用——以横向误差和航向误差为状态，转向角为控制量，建立线性化状态空间模型，设计LQR控制器。理解Q/R权重矩阵的物理意义（Q大→跟踪紧但控制大，R大→控制平缓但跟踪松）。

**为什么学**：LQR是自动驾驶横向控制的经典方案，很多量产车辆的横向控制器就是LQR或LQR+前馈。你的现代控制理论基础（状态空间、Riccati方程、能控能观性）使你能深入理解和设计LQR控制器。

**学到什么程度**：能将车辆横向误差模型转化为标准LQR问题，求解Riccati方程获得最优反馈增益K，能用Python实现LQR横向控制器并与纯追踪/MPC对比。

**学习时间**：2-3天

**推荐资源**：
- 书籍：你的现代控制理论教材中LQR章节（如刘豹《现代控制理论》或郑大钟《线性系统理论》）
- 论文：搜索"LQR lateral control autonomous vehicle"
- GitHub：`AtsushiSakai/PythonRobotics` 的LQR模块
- 实践：Apollo的控制模块 `apollo/modules/control/controller/lat_controller.cc` 中有LQR实现

**检验标准**：能推导横向误差状态空间模型，手算2x2系统的Riccati方程解，能实现LQR控制器并在仿真中验证。

### 7.3 Stanley控制器与几何控制思想

**学什么**：Stanley控制器的推导——从前端点横向误差几何关系出发，推导出转向角公式 delta = theta_e + arctan(k * e / v)。理解其与经典控制中"比例+前馈"结构的对应关系。理解速度趋于零时的奇异性和处理方法。

**为什么学**：Stanley是几何方法和控制理论的桥梁。通过分析Stanley，你能看到一个看似简单的几何控制器背后其实是Lyapunov稳定性分析的结果。

**学到什么程度**：能用Lyapunov方法证明Stanley控制器的稳定性（横向误差收敛），能修改增益参数并分析收敛速度。

**学习时间**：1-2天

**推荐资源**：
- 论文：Hoffmann 2007 (同Part 4.3)
- 博客：搜索"Stanley controller Lyapunov stability proof"

**检验标准**：能给出Stanley控制器的Lyapunov稳定性分析过程，能对比Stanley和LQR在不同工况下的性能。

---

## 总体学习路线建议（时间线参考）

| 阶段 | 时间 | 内容 |
|------|------|------|
| 第1-2周 | 入门期 | Part 1 全局规划 (Dijkstra, A*, RRT, RRT*) + Part 2 局部规划 (DWA, APF) |
| 第3-4周 | 核心期 | Part 3 运动规划 (Frenet, 多项式, Lattice, QP) |
| 第5-6周 | 深入期 | Part 4 MPC (车辆模型, MPC控制器, CasADi/OSQP) |
| 第7周 | 补充期 | Part 5 行为决策 + Part 6 Nav2 |
| 第8周 | 整合期 | Part 7 控制应用 + 全链路集成项目 |

**推荐集成项目**：在Gazebo/Carla仿真中实现完整pipeline——Nav2全局规划(A*) + Lattice局部规划 + MPC跟踪控制 + 行为树决策，作为研究生入学前的portfolio。

---

## 核心GitHub仓库汇总

| 仓库 | 用途 |
|------|------|
| `AtsushiSakai/PythonRobotics` | 几乎所有算法的Python参考实现，含可视化 |
| `zhm-real/PathPlanning` | 全局/局部规划算法集锦 |
| `OMPL/ompl` | 基于采样规划的标准库 |
| `casadi/casadi` | MPC/优化建模工具 |
| `osqp/osqp` | QP求解器 |
| `ros-navigation/navigation2` | Nav2官方仓库 |
| `BehaviorTree/BehaviorTree.CPP` | 行为树框架 |

---

以上指南以"能工程落地"为核心导向，每个技术点都给出了明确的学习深度标准和检验方法。建议以PythonRobotics仓库为主线边学边练，逐步过渡到C++和ROS2工程实践。

---

# 模块六：工程部署、仿真环境、项目实战与求职指南

# 自动驾驶方向 - 工程部署、仿真环境、项目实战与求职指南

> 适用对象：机器人工程本科背景、985船舶与海洋工程硕士在读（2029年毕业）
> 导师合作方：云洲智能（无人船头部）、字节跳动
> 核心目标：硕士期间积累自动驾驶/机器人通用技术栈，毕业时跳脱海洋行业

---

## Part 1: ROS2 完全指南

### 1.1 ROS2 vs ROS1 核心区别

**学什么**：理解 ROS2 相对于 ROS1 的架构升级，包括通信机制、实时性、安全性等方面的根本变化。

**为什么学**：ROS1 已于 2025 年 5 月正式 EOL（End of Life），工业界和学术界全面转向 ROS2。自动驾驶公司（Apollo 除外）几乎全部基于 ROS2 或其变体开发。你做无人船项目大概率也需要 ROS2。

**学到什么程度**：能清晰说出 ROS2 相比 ROS1 的 5 个以上核心改进，理解为什么这些改进对实时系统和生产环境至关重要。

**学习时间**：2-3 天（纯理论理解）

**核心区别一览**：

| 维度 | ROS1 | ROS2 |
|------|------|------|
| 通信中间件 | 自定义 TCP/UDP（roscomm） | DDS（Data Distribution Service） |
| 实时性 | 不支持 | 支持（QoS 策略可配置） |
| 去中心化 | 依赖 rosmaster（单点故障） | 完全去中心化（DDS Discovery） |
| 多机通信 | 需要复杂配置 | 原生支持（DDS 天然支持） |
| 生命周期管理 | 无 | Lifecycle Node |
| 安全机制 | 无 | SROS2（DDS Security） |
| 构建系统 | catkin | ament + colcon |
| Python | Python 2/3 混乱 | Python 3.8+ |
| 跨平台 | 主要 Linux | Linux/macOS/Windows |
| 进程内通信 | 不支持 | 支持（Intra-process Communication） |
| 启动文件 | XML（.launch） | Python/YAML/XML（推荐 Python） |

**通信模型对比**：

- ROS1：Master 节点注册机制，Topic 通过 TCPROS/UDPROS 传输
- ROS2：基于 DDS 的 Publish-Subscribe 模型，无需 Master，节点通过 DDS Discovery 自动发现

**推荐资源**：
- 官方文档 ROS2 vs ROS1：https://docs.ros.org/en/jazzy/Concepts/Advanced/About-ROS-2-Communication.html
- 古月居 ROS2 教程（中文）：https://www.guyuehome.com/ (搜索 ROS2 系列)
- 书籍：《ROS2智能机器人开发实践》 胡春旭 著（古月居作者）

**检验标准**：能画出 ROS2 基于 DDS 的通信架构图，解释为什么去掉 rosmaster 是必要的。

---

### 1.2 ROS2 安装与环境配置

**学什么**：在 Ubuntu 系统上安装 ROS2，配置开发环境，理解不同发行版的区别。

**为什么学**：这是所有后续学习的基础。不同发行版的 API 有细微差异，选错版本会导致兼容性问题。

**学到什么程度**：能独立完成安装、环境配置、创建第一个工作空间。

**学习时间**：1 天

**版本选择建议**：

| 版本 | Ubuntu | 状态 | 建议 |
|------|--------|------|------|
| Humble (LTS) | 22.04 | 长期支持到 2027.5 | **强烈推荐，生态最成熟** |
| Iron | 22.04 | 2024.11 EOL | 不推荐 |
| Jazzy (LTS) | 24.04 | 长期支持到 2029.5 | 新项目可选，但部分包兼容性待验证 |
| Rolling | 最新 | 持续更新 | 仅用于测试 |

**建议**：使用 **ROS2 Humble**，它是当前自动驾驶社区（Autoware、Nav2 等）最广泛支持的版本。

**安装步骤（Ubuntu 22.04 + Humble）**：

```bash
# 1. 设置locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 设置源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop    # 包含rviz2, rqt等

# 4. 环境配置
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. 安装开发工具
sudo apt install python3-colcon-common-extensions python3-rosdep2
sudo rosdep init
rosdep update

# 6. 安装Gazebo（后续仿真用）
sudo apt install ros-humble-gazebo-ros-pkgs
```

**开发工具推荐**：
- IDE：VS Code + ROS 扩展包（ms-iot.vscode-ros）或 CLion
- 终端工具：terminator（多窗口终端）
- 版本管理：git + GitHub

**推荐资源**：
- 官方安装文档：https://docs.ros.org/en/humble/Installation.html
- 古月居安装教程：https://www.guyuehome.com/ (ROS2入门系列)
- 鱼香ROS一键安装：https://fishros.com/ （提供一键安装脚本，适合快速上手）

**检验标准**：运行 `ros2 run demo_nodes_cpp talker` 和 `ros2 run demo_nodes_py listener`，能正常通信。

---

### 1.3 核心概念

#### 1.3.1 节点（Node）

**学什么**：ROS2 的基本执行单元——节点。理解节点的生命周期、通信接口。

**为什么学**：自动驾驶系统中，感知、预测、规划、控制通常各自运行在不同节点中。节点是系统拆分的最小单位。

**学到什么程度**：能用 C++ 和 Python 分别编写节点，理解 NodeOptions、回调组（Callback Group）。

**学习时间**：2 天

**核心概念**：
- 节点是 ROS2 中的最小执行单元
- 一个进程可以包含多个节点（Component 模式）
- 节点通过 Topic/Service/Action 与其他节点通信
- 每个节点有唯一名称，支持命名空间（Namespace）

**C++ 节点示例**：

```cpp
#include "rclcpp/rclcpp.hpp"

class MinimalNode : public rclcpp::Node {
public:
    MinimalNode() : Node("minimal_node"), count_(0) {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Hello %d", count_++);
            });
    }
private:
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalNode>());
    rclcpp::shutdown();
    return 0;
}
```

**回调组（Callback Group）**：
- `MutuallyExclusive`：回调函数互斥执行（默认），同一时间只能有一个回调
- `Reentrant`：回调函数可并发执行
- 对于实时系统，理解回调组对线程安全至关重要

**推荐资源**：
- 官方教程 Writing a simple publisher and subscriber：https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
- 《ROS2智能机器人开发实践》第3章

**检验标准**：能独立写一个 C++ 节点，使用回调组控制并发行为。

---

#### 1.3.2 话题（Topic）

**学什么**：ROS2 的发布-订阅通信模型。这是最常用的通信方式。

**为什么学**：自动驾驶中，传感器数据（图像、点云、IMU）几乎全部通过 Topic 传递。

**学到什么程度**：能实现 Publisher 和 Subscriber，理解 QoS 对通信可靠性的影响。

**学习时间**：2 天

**核心特点**：
- 异步通信，发布者和订阅者解耦
- 一个 Topic 可以有多个发布者和多个订阅者
- 基于 DDS 的 Publish-Subscribe 模型
- 支持多种 QoS 策略

**C++ Publisher 示例**：

```cpp
class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("minimal_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                auto message = std_msgs::msg::String();
                message.data = "Hello, world! " + std::to_string(count_++);
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                publisher_->publish(message);
            });
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};
```

**常用命令**：
```bash
ros2 topic list                    # 列出所有话题
ros2 topic info /topic_name        # 查看话题信息
ros2 topic echo /topic_name        # 打印话题数据
ros2 topic hz /topic_name          # 查看发布频率
ros2 topic pub /topic_name ...     # 手动发布
```

**推荐资源**：
- 官方 Topic 教程：https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

**检验标准**：能用 C++ 写一个图像发布节点和接收节点，设置合理的 QoS。

---

#### 1.3.3 服务（Service）

**学什么**：ROS2 的请求-响应通信模型（同步 RPC）。

**为什么学**：适合需要确认结果的操作，如查询地图、触发校准、发送控制指令等。

**学到什么程度**：能自定义 .srv 文件，实现 Service Server 和 Client。

**学习时间**：1 天

**核心特点**：
- 同步通信：客户端发送请求，等待服务端响应
- 一对多：一个服务端可以服务多个客户端
- 请求-响应模式，适合查询类和命令类操作

**Service 示例**：

```srv
# 自定义 srv 文件：AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

**常用命令**：
```bash
ros2 service list                    # 列出所有服务
ros2 service type /service_name      # 查看服务类型
ros2 service call /service_name ...  # 手动调用服务
```

**推荐资源**：
- 官方 Service 教程：https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html

**检验标准**：能自定义 srv 文件，实现 Server 和 Client，理解同步调用的阻塞特性。

---

#### 1.3.4 动作（Action）

**学什么**：ROS2 的长时任务通信模型，结合了 Topic 和 Service 的特点。

**为什么学**：自动驾驶中，路径规划、导航目标设定等长时任务适合用 Action。Nav2 大量使用 Action 接口。

**学到什么程度**：能自定义 .action 文件，实现 Action Server 和 Client，理解反馈机制。

**学习时间**：1-2 天

**核心特点**：
- 目标（Goal）：客户端发送任务目标
- 反馈（Feedback）：服务端周期性发送进度
- 结果（Result）：任务完成后返回结果
- 支持取消（Cancel）

**Action 文件示例**：

```
# Fibonacci.action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

**Nav2 中的 Action**：
- `NavigateToPose`：导航到目标点（带反馈：距离、时间估计）
- `FollowPath`：跟踪路径
- `ComputePathToPose`：计算路径

**推荐资源**：
- 官方 Action 教程：https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-An-Action-Server-Client/Cpp.html
- Nav2 Action 接口文档：https://navigation.ros.org/

**检验标准**：理解 Action 的 Goal/Feedback/Result 三段式通信，能在 Nav2 中正确发送导航目标。

---

#### 1.3.5 参数（Parameter）与 Launch 文件

**学什么**：ROS2 的参数系统和 Launch 启动系统。

**为什么学**：自动驾驶系统包含大量可配置参数（检测阈值、规划器参数等），Launch 文件用于编排多节点启动。

**学到什么程度**：能使用 YAML 配置参数，用 Python 编写复杂的 Launch 文件。

**学习时间**：2 天

**参数系统**：
```bash
ros2 param list                       # 列出参数
ros2 param get /node_name param_name  # 获取参数值
ros2 param set /node_name param_name value  # 设置参数
ros2 param dump /node_name            # 导出参数
ros2 param load /node_name params.yaml      # 加载参数
```

**YAML 参数文件**：
```yaml
/**:
  ros__parameters:
    use_sim_time: true
    detection_threshold: 0.5
    max_speed: 2.0
```

**Python Launch 文件**：

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        Node(
            package='my_package',
            executable='perception_node',
            name='perception',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'threshold': 0.5,
            }],
            remappings=[
                ('/input/image', '/camera/image_raw'),
            ],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'navigation_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
```

**推荐资源**：
- 官方 Launch 教程：https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html
- Nav2 的 Launch 文件：https://github.com/ros-planning/navigation2/tree/humble/nav2_bringup/launch

**检验标准**：能编写一个包含 5 个以上节点的 Launch 文件，带参数配置和条件启动。

---

#### 1.3.6 QoS 策略（Quality of Service）

**学什么**：ROS2 的 QoS 策略配置。这是 ROS2 相比 ROS1 最重要的改进之一，对实时系统尤为关键。

**为什么学**：自动驾驶系统中，传感器数据需要低延迟（Best Effort），控制指令需要高可靠（Reliable）。QoS 配置不当会导致通信失败或性能下降。面试高频题。

**学到什么程度**：能根据场景选择合适的 QoS 策略组合，理解每个策略的含义和影响。

**学习时间**：1-2 天

**QoS 策略详解**：

| 策略 | 选项 | 说明 | 自动驾驶场景 |
|------|------|------|-------------|
| Reliability | RELIABLE / BEST_EFFORT | 可靠传输 vs 尽力传输 | 图像→BEST_EFFORT，控制→RELIABLE |
| Durability | VOLATILE / TRANSIENT_LOCAL | 是否保留最后一条消息 | 静态TF→TRANSIENT_LOCAL |
| History | KEEP_LAST(n) / KEEP_ALL | 保留多少条历史 | 传感器→KEEP_LAST(5) |
| Deadline | Duration | 预期的消息到达周期 | 100Hz 传感器→10ms |
| Lifespan | Duration | 消息有效期 | 实时传感器→短 |
| Liveliness | AUTOMATIC / MANUAL | 存活声明方式 | 安全关键→MANUAL |

**常见 QoS 组合**：

```cpp
// 传感器数据（图像、点云）
rclcpp::SensorDataQoS();  // BEST_EFFORT, VOLATILE, KEEP_LAST(5)

// 控制指令
rclcpp::QoS(10)
    .reliable()
    .durability_volatile();

// 地图、TF等配置数据
rclcpp::QoS(10)
    .reliable()
    .transient_local();
```

**QoS 兼容性规则**：
- Publisher 和 Subscriber 的 Reliability 必须兼容：RELIABLE 可以和 RELIABLE 或 BEST_EFFORT 通信，但 BEST_EFFORT 只能和 BEST_EFFORT 通信（实际上 RELIABLE Publisher 可以发给 BEST_EFFORT Subscriber，但反过来不行）
- Durability 必须兼容

**推荐资源**：
- 官方 QoS 文档：https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- DDS QoS 官方规范：https://www.omg.org/spec/DDS/
- 博客：ROS2 QoS 完全指南（搜索 "ROS2 QoS tutorial"）

**检验标准**：能解释为什么相机话题用 BEST_EFFORT 而不用 RELIABLE，画出 QoS 兼容性矩阵。

---

#### 1.3.7 生命周期节点（Lifecycle Node）

**学什么**：ROS2 的节点生命周期管理机制。

**为什么学**：自动驾驶系统要求节点状态可控（未配置→未激活→激活→停用→清理→关闭）。Nav2 的所有核心节点都是 Lifecycle Node。

**学到什么程度**：能实现自定义 Lifecycle Node，理解每个状态转换的回调。

**学习时间**：1 天

**状态机**：

```
Unconfigured → Inactive → Active
    ↑              ↑         ↓
    └──────────────┴── Inactive
    ↑                        ↓
  Finalized ←───────────── CleaningUp
```

**状态转换回调**：
- `on_configure`：加载参数、创建通信接口
- `on_activate`：开始处理数据
- `on_deactivate`：暂停处理
- `on_cleanup`：释放资源，回到未配置状态
- `on_shutdown`：关闭节点

**常用命令**：
```bash
ros2 lifecycle get /node_name              # 获取当前状态
ros2 lifecycle set /node_name configure    # 触发转换
ros2 lifecycle set /node_name activate
```

**推荐资源**：
- 官方 Lifecycle 教程：https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-A-Composable-Node.html
- Nav2 Lifecycle 管理：https://navigation.ros.org/configuration/packages/configuring-bt-plugins.html

**检验标准**：能实现一个 Lifecycle Node，理解 Nav2 中为什么所有节点都要用 Lifecycle 管理。

---

### 1.4 自定义消息/服务/动作定义

**学什么**：使用 .msg、.srv、.action 文件定义自定义接口。

**为什么学**：自动驾驶项目几乎不可避免地需要自定义消息类型（如检测结果、规划路径等）。

**学到什么程度**：能独立定义接口文件，配置 CMakeLists.txt 和 package.xml，正确编译和使用。

**学习时间**：1 天

**目录结构**：
```
my_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── DetectedObject.msg
│   └── DetectedObjects.msg
├── srv/
│   └── GetMap.srv
└── action/
    └── Navigate.action
```

**DetectedObject.msg 示例**：
```
std_msgs/Header header
string class_name
float32 confidence
float32[8] bbox_3d    # 8个顶点的xyz坐标
geometry_msgs/Pose pose
float32[3] dimensions # 长宽高
```

**CMakeLists.txt 关键配置**：
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
  "srv/GetMap.srv"
  "action/Navigate.action"
  DEPENDENCIES std_msgs geometry_msgs
)
```

**package.xml 添加依赖**：
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**推荐资源**：
- 官方自定义接口教程：https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html

**检验标准**：能定义一个包含 DetectedObject[] 数组的消息类型，成功编译并在节点中使用。

---

### 1.5 tf2 坐标变换

**学什么**：ROS2 的坐标变换系统 tf2。这是 SLAM 和导航的基础，必须熟练掌握。

**为什么学**：自动驾驶系统涉及大量坐标系（车体、传感器、地图、世界）。tf2 负责管理这些坐标系之间的变换关系。SLAM、定位、感知、规划都依赖 tf2。

**学到什么程度**：能正确配置和使用 tf2，理解坐标变换链，能调试常见 tf 问题。

**学习时间**：3-4 天（含实操）

#### 1.5.1 核心概念

**坐标系树**：
```
map → odom → base_link → base_lidar
                       → base_camera
                       → base_imu
```

- `map`：全局地图坐标系（固定）
- `odom`：里程计坐标系（会漂移）
- `base_link`：车体坐标系
- 各传感器坐标系

**变换数据结构**：
- `geometry_msgs/TransformStamped`：包含平移（translation）和旋转（rotation，四元数表示）
- 时间戳：变换有时间属性，支持查询历史变换

#### 1.5.2 静态变换发布

```cpp
// 发布 base_link 到 lidar 的静态变换
auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

geometry_msgs::msg::TransformStamped t;
t.header.stamp = node->now();
t.header.frame_id = "base_link";
t.child_frame_id = "base_lidar";
t.transform.translation.x = 1.0;
t.transform.translation.y = 0.0;
t.transform.translation.z = 0.5;
t.transform.rotation.w = 1.0;

static_broadcaster->sendTransform(t);
```

**命令行方式**：
```bash
ros2 run tf2_ros static_transform_publisher 1.0 0 0.5 0 0 0 base_link base_lidar
```

#### 1.5.3 动态变换发布

```cpp
// 发布 odom 到 base_link 的动态变换（由里程计提供）
auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

geometry_msgs::msg::TransformStamped t;
t.header.stamp = node->now();
t.header.frame_id = "odom";
t.child_frame_id = "base_link";
t.transform.translation.x = odom_x;
t.transform.translation.y = odom_y;
t.transform.rotation = odom_quat;

broadcaster->sendTransform(t);
```

#### 1.5.4 tf2 监听与查询

```cpp
auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

// 查询 base_link 在 map 坐标系下的位姿
try {
    geometry_msgs::msg::TransformStamped transform = 
        tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    
    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;
} catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node->get_logger(), "Transform error: %s", ex.what());
}
```

**常用命令**：
```bash
ros2 run tf2_tools view_frames        # 生成坐标系树PDF
ros2 topic echo /tf                    # 查看动态变换
ros2 topic echo /tf_static             # 查看静态变换
ros2 run tf2_ros tf2_echo frame1 frame2  # 查看两坐标系间变换
```

**常见问题与调试**：
- "Could not find a connection between frames"：坐标系树不连通
- 时间戳不匹配：确保使用 `use_sim_time:=true`
- 变换方向搞反：`lookupTransform("A", "B")` 返回的是 B 在 A 中的坐标

**推荐资源**：
- 官方 tf2 教程：https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
- 书籍：《ROS2智能机器人开发实践》坐标变换章节

**检验标准**：能搭建完整的坐标系树（map→odom→base_link→sensor），用 rviz2 可视化验证。能解释 lookupTransform 中源坐标系和目标坐标系的含义。

---

### 1.6 常用工具

#### 1.6.1 rviz2 可视化

**学什么**：ROS2 的三维可视化工具。

**为什么学**：调试 SLAM、导航、感知结果的必备工具。面试中可能会问你如何调试一个定位问题，答案往往涉及 rviz2。

**学到什么程度**：能熟练添加各种显示类型（点云、图像、TF、路径、标记等），能用 rviz2 调试问题。

**学习时间**：2 天

**常用显示类型**：
| 显示类型 | Topic | 用途 |
|---------|-------|------|
| PointCloud2 | /points_raw | LiDAR 点云 |
| Image | /camera/image_raw | 相机图像 |
| LaserScan | /scan | 2D 激光雷达 |
| Map | /map | 占据栅格地图 |
| Path | /plan | 规划路径 |
| Marker / MarkerArray | /markers | 自定义可视化 |
| Pose / PoseArray | /poses | 位姿显示 |
| TF | - | 坐标系变换 |
| RobotModel | - | 机器人 URDF 模型 |

**Marker 示例（可视化检测框）**：
```cpp
visualization_msgs::msg::Marker marker;
marker.header.frame_id = "base_link";
marker.header.stamp = node->now();
marker.ns = "detection";
marker.id = 0;
marker.type = visualization_msgs::msg::Marker::CUBE;
marker.action = visualization_msgs::msg::Marker::ADD;
marker.pose.position.x = 5.0;
marker.pose.position.y = 2.0;
marker.scale.x = 1.5;
marker.scale.y = 0.8;
marker.scale.z = 1.5;
marker.color.r = 1.0;
marker.color.a = 0.5;
marker.lifetime = rclcpp::Duration::from_seconds(0.5);
```

**推荐资源**：
- 官方 rviz2 文档：https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-URDF-With-Robot-State-Publisher.html
- rviz2 用户指南：http://wiki.ros.org/rviz2

**检验标准**：能在 rviz2 中同时显示点云、检测框、规划路径和 TF 坐标系。

---

#### 1.6.2 rqt 工具集

**学什么**：ROS2 的图形化调试工具集。

**学到什么程度**：熟练使用 rqt_graph、rqt_console、rqt_plot。

**学习时间**：半天

**核心工具**：
```bash
ros2 run rqt_graph rqt_graph    # 节点通信图
ros2 run rqt_console rqt_console  # 日志查看
ros2 run rqt_plot rqt_plot      # 数据绘图
ros2 run rqt_tf_tree rqt_tf_tree  # TF树
```

---

#### 1.6.3 ros2 bag 数据录制与回放

**学什么**：ROS2 的数据录制和回放工具。

**为什么学**：自动驾驶开发中，需要录制传感器数据用于离线调试和算法验证。ros2 bag 是最常用的数据格式。

**学到什么程度**：能录制、回放、查询 bag 文件，理解与 rosbag1 的区别（使用 SQLite3/mcap 存储）。

**学习时间**：半天

**基本命令**：
```bash
# 录制所有话题
ros2 bag record -a -o my_recording

# 录制指定话题
ros2 bag record /camera/image_raw /points_raw /imu/data -o sensor_data

# 回放
ros2 bag play my_recording/

# 回放并循环
ros2 bag play my_recording/ --loop

# 查看 bag 信息
ros2 bag info my_recording/
```

**MCAP 格式（推荐）**：
```bash
# 安装 mcap 存储插件
sudo apt install ros-humble-rosbag2-storage-mcap

# 使用 mcap 格式录制
ros2 bag record -a -o data --storage mcap
```

**推荐资源**：
- 官方 rosbag2 文档：https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html
- mcap 格式文档：https://mcap.dev/

**检验标准**：能录制包含相机和 LiDAR 数据的 bag 文件，在另一个终端中回放并在 rviz2 中可视化。

---

#### 1.6.4 colcon 构建工具

**学什么**：ROS2 的构建工具 colcon。

**学到什么程度**：能正确构建多包工作空间，理解构建顺序和依赖管理。

**学习时间**：半天

**基本命令**：
```bash
# 构建所有包
colcon build

# 构建指定包
colcon build --packages-select my_package

# 构建并行
colcon build --parallel-workers 4

# 带 symlink（Python 包修改后无需重新构建）
colcon build --symlink-install

# 构建后 source
source install/setup.bash

# 测试
colcon test
colcon test-result --verbose
```

**推荐资源**：
- colcon 官方文档：https://colcon.readthedocs.io/

**检验标准**：能在多包工作空间中正确构建、测试，理解依赖关系。

---

### 1.7 ROS2 性能优化

#### 1.7.1 共享内存传输（Intra-process Communication）

**学什么**：ROS2 的进程内通信机制，避免数据序列化/反序列化开销。

**为什么学**：自动驾驶系统对延迟敏感。图像数据（几 MB）通过 DDS 网络传输有显著开销，共享内存可将延迟降低一个数量级。

**学到什么程度**：能配置和使用 Intra-process Communication，理解 Component 模式。

**学习时间**：1-2 天

**使用 Component 模式**：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class MyComponent : public rclcpp::Node {
public:
    explicit MyComponent(const rclcpp::NodeOptions & options)
    : Node("my_component", options) {
        // 使用 Intra-process Communication
        rclcpp::PublisherOptions pub_options;
        pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::ENABLE;
        
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "output_image", 10, pub_options);
        
        rclcpp::SubscriptionOptions sub_options;
        sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::ENABLE;
        
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "input_image", 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                // 零拷贝处理
                process_image(msg);
                publisher_->publish(std::move(msg));
            }, sub_options);
    }
    // ...
};

RCLCPP_COMPONENTS_REGISTER_NODE(MyComponent)
```

**启动 Component**：
```xml
<node_container pkg="rclcpp_components" exec="component_container" name="my_container">
    <composable_node pkg="my_pkg" plugin="MyComponent::MyComponent" name="comp1"/>
    <composable_node pkg="my_pkg" plugin="MyComponent::MyComponent" name="comp2"/>
</node_container>
```

**推荐资源**：
- 官方 Intra-process 通信文档：https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-A-Composable-Node.html
- ros2/examples 仓库中的 composable_node 示例

**检验标准**：能用 Component 模式部署两个节点到同一进程，验证零拷贝通信。

---

#### 1.7.2 DDS 选型

**学什么**：ROS2 底层 DDS 中间件的选型与配置。

**为什么学**：DDS 是 ROS2 的通信基石，不同 DDS 实现在性能、稳定性上有差异。生产环境中需要根据场景选择合适的 DDS。

**学到什么程度**：了解主要 DDS 实现的特点，能进行基本的 DDS 配置和性能调优。

**学习时间**：1 天

**DDS 实现对比**：

| DDS 实现 | 开发者 | 特点 | 推荐场景 |
|---------|--------|------|---------|
| FastDDS | eProsima | 功能丰富，社区活跃 | 默认选择，通用场景 |
| CycloneDDS | Eclipse | 性能优异，内存占用低 | 高性能需求 |
| ConnextDDS | RTI | 商用级，工业认证 | 商业项目 |
| GurumDDS | Gurum | 韩国厂商 | 不推荐 |

**切换 DDS**：
```bash
# 安装 CycloneDDS
sudo apt install ros-humble-rmw-cyclonedds-cpp

# 切换到 CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 验证
ros2 doctor --report
```

**DDS 配置文件（FastDDS XML）**：
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_descriptor_id>
            <type>UDPv4</type>
            <maxMessageSize>65500</maxMessageSize>
            <sendBufferSize>65536</sendBufferSize>
            <receiveBufferSize>65536</receiveBufferSize>
        </transport_descriptor>
    </transport_descriptors>
</profiles>
```

**推荐资源**：
- ROS2 DDS 文档：https://docs.ros.org/en/humble/Concepts/Intermediate/About-Middleware.html
- FastDDS 官方文档：https://fast-dds.docs.eprosima.com/
- CycloneDDS 文档：https://cyclonedds.io/

**检验标准**：能在 FastDDS 和 CycloneDDS 之间切换，用 benchmark 测试传输延迟差异。

---

## Part 2: 仿真环境

### 2.1 Gazebo

**学什么**：Gazebo 机器人仿真器，用于搭建无人车/USV 仿真环境，进行传感器仿真和算法验证。

**为什么学**：Gazebo 是 ROS2 生态中最主流的仿真器，Nav2、Autoware 都依赖 Gazebo 进行仿真测试。USV 仿真更是你的核心需求。

**学到什么程度**：能独立搭建包含多种传感器的仿真环境，理解物理引擎参数调优。

**学习时间**：2-3 周

#### 2.1.1 版本选择

| 版本 | 说明 | 建议 |
|------|------|------|
| Gazebo Classic (11) | 老版本，生态最成熟 | 部分老教程使用，逐步淘汰 |
| Gazebo Fortress (Ignition) | 新架构，ROS2 Humble 默认支持 | **当前推荐** |
| Gazebo Harmonic | 最新版，Jazzy+ 支持 | 新项目可选 |

**建议**：使用 Gazebo Fortress（Ignition Gazebo），它是 ROS2 Humble 的默认搭配。

#### 2.1.2 搭建仿真环境

**基本安装**：
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros-gz
```

**世界文件（SDF 格式）**：
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="ocean_world">
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-sensors-system" name="gz::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </world>
</sdf>
```

**推荐资源**：
- Gazebo 官方教程：https://gazebosim.org/docs
- ROS2 + Gazebo 集成：https://gazebosim.org/docs/garden/ros2_integration
- 《ROS2智能机器人开发实践》仿真章节

#### 2.1.3 传感器仿真

**相机**：
```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>1920</width>
      <height>1080</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
</sensor>
```

**LiDAR**：
```xml
<sensor name="lidar" type="gpu_lidar">
  <lidar>
    <scan>
      <horizontal>
        <samples>1024</samples>
        <resolution>1</resolution>
        <min_angle>-3.14</min_angle>
        <max_angle>3.14</max_angle>
      </horizontal>
      <vertical>
        <samples>64</samples>
        <min_angle>-0.26</min_angle>
        <max_angle>0.26</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.5</min>
      <max>100</max>
      <resolution>0.01</resolution>
    </range>
  </lidar>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
</sensor>
```

**IMU**：
```xml
<sensor name="imu" type="imu">
  <always_on>1</always_on>
  <update_rate>200</update_rate>
</sensor>
```

**GPS**：
```xml
<sensor name="gps" type="navsat">
  <always_on>1</always_on>
  <update_rate>10</update_rate>
</sensor>
```

#### 2.1.4 物理引擎参数调优

**DART 引擎参数**：
- `max_step_size`：仿真步长，越小越精确但越慢（0.001s 是常用值）
- `real_time_factor`：实时因子，1.0 表示实时
- `gravity`：重力加速度
- 接触参数：摩擦系数、弹性系数、阻尼系数

**水下/水面仿真**：
- 需要自定义浮力插件
- 水动力学参数（阻力、附加质量）
- 可参考 UUV Simulator：https://uuvsimulator.github.io/

**推荐资源**：
- Gazebo 传感器文档：https://gazebosim.org/api/sensors/8/sensorelements.html
- UUV Simulator（水下仿真）：https://github.com/uuvsimulator/uuv_simulator
- boat_simulator（水面仿真）：搜索 GitHub "USV simulator ROS2"

**检验标准**：能搭建一个包含 USV 模型、相机、LiDAR、IMU、GPS 的 Gazebo 仿真环境，并通过 ROS2 Topic 接收传感器数据，在 rviz2 中可视化。

---

### 2.2 CARLA 自动驾驶仿真

**学什么**：CARLA 是目前自动驾驶行业最主流的开源仿真器之一，提供高逼真城市场景、真实感渲染、交通流生成等。

**为什么学**：进入自动驾驶公司的核心技能之一。小鹏、蔚来、百度等公司都使用 CARLA 或类似仿真器进行算法验证。CARLA 也是学术界 benchmark 的标准平台。

**学到什么程度**：能独立搭建仿真场景、配置传感器、生成交通流、与 ROS2 对接。

**学习时间**：2-3 周

#### 2.2.1 安装与配置

**安装 CARLA（0.9.15 或最新）**：
```bash
# 下载 CARLA
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.15.tar.gz
tar -xzf CARLA_0.9.15.tar.gz
cd CARLA_0.9.15

# 安装 Python API
pip install carla==0.9.15

# 启动 CARLA 服务器
./CarlaUE4.sh -quality-level=Epic
```

**ROS2 Bridge 安装**：
```bash
# 方式1：使用官方 ros-bridge
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git -b ros2
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

#### 2.2.2 场景搭建

**Python API 基本用法**：
```python
import carla

# 连接CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# 加载地图
world = client.load_world('Town03')

# 设置天气
weather = carla.WeatherParameters(
    cloudiness=30.0,
    precipitation=0.0,
    sun_altitude_angle=70.0
)
world.set_weather(weather)
```

#### 2.2.3 传感器配置

```python
# 添加RGB相机
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '1920')
camera_bp.set_attribute('image_size_y', '1080')
camera_bp.set_attribute('fov', '90')
camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

# 添加LiDAR
lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('channels', '64')
lidar_bp.set_attribute('points_per_second', '1200000')
lidar_bp.set_attribute('range', '100')
lidar_transform = carla.Transform(carla.Location(x=0, z=2.5))
lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

# 添加IMU
imu_bp = world.get_blueprint_library().find('sensor.other.imu')
imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)

# 添加GPS
gps_bp = world.get_blueprint_library().find('sensor.other.gnss')
gps = world.spawn_actor(gps_bp, carla.Transform(), attach_to=vehicle)
```

#### 2.2.4 交通流生成

```python
# 生成交通流
traffic_manager = client.get_traffic_manager()
traffic_manager.set_global_distance_to_leading_vehicle(2.5)

# 批量生成车辆
blueprints = world.get_blueprint_library().filter('vehicle.*')
spawn_points = world.get_map().get_spawn_points()

vehicles = []
for i in range(50):
    blueprint = random.choice(blueprints)
    vehicle = world.try_spawn_actor(blueprint, random.choice(spawn_points))
    if vehicle:
        vehicle.set_autopilot(True)
        vehicles.append(vehicle)

# 生成行人
walker_bp = world.get_blueprint_library().find('walker.pedestrian.*')
```

#### 2.2.5 与 ROS2 桥接

**启动 ROS2 Bridge**：
```bash
# 启动 CARLA 服务器
./CarlaUE4.sh

# 启动 ROS2 Bridge
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py

# 添加传感器
ros2 run carla_ros_bridge carla_spawn_objects --ros-args -p objects_definition_file:=/path/to/objects.json
```

**objects.json 示例**：
```json
{
    "objects": [
        {
            "type": "vehicle.tesla.model3",
            "id": "ego_vehicle",
            "sensors": [
                {
                    "type": "sensor.camera.rgb",
                    "id": "front_camera",
                    "spawn_point": {"x": 1.5, "y": 0.0, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                    "image_size_x": 1920,
                    "image_size_y": 1080
                },
                {
                    "type": "sensor.lidar.ray_cast",
                    "id": "lidar",
                    "spawn_point": {"x": 0.0, "y": 0.0, "z": 2.5, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                    "channels": 64,
                    "points_per_second": 1200000
                }
            ]
        }
    ]
}
```

**推荐资源**：
- CARLA 官方文档：https://carla.readthedocs.io/
- CARLA ROS2 Bridge：https://github.com/carla-simulator/ros-bridge
- CARLA 官方教程 Python API：https://carla.readthedocs.io/en/latest/tutorials/
- B站搜索 "CARLA 自动驾驶仿真" 有大量中文教程

**检验标准**：能在 CARLA 中生成一辆带相机和 LiDAR 的车辆，通过 ROS2 Bridge 在 rviz2 中看到传感器数据，同时有交通流运行。

---

### 2.3 LGSVL（了解）

**学什么**：了解 LGSVL 的历史地位和停维护的现状。

**为什么学**：LGSVL 曾经是自动驾驶仿真的重要平台，2022 年停止维护。了解它有助于理解自动驾驶仿真技术的发展脉络。

**核心信息**：
- 开发者：LG Silicon Valley Lab
- 2022 年停止维护
- 高保真渲染，支持 Apollo 和 Autoware
- 替代方案：CARLA、NVIDIA DRIVE Sim

---

### 2.4 NVIDIA Isaac Sim（了解）

**学什么**：了解 NVIDIA 的机器人仿真平台。

**为什么学**：基于 Omniverse 的高保真仿真器，支持 PhysX 物理引擎，与 ROS2 深度集成。在机器人领域（非自动驾驶）有应用。

**核心信息**：
- 基于 NVIDIA Omniverse
- 高保真渲染和物理仿真
- 支持 ROS2 桥接
- 主要面向机器人开发
- 安装较大（需要 NVIDIA GPU，建议 RTX 系列）

**参考**：
- Isaac Sim 文档：https://docs.omniverse.nvidia.com/isaacsim/latest/

---

### 2.5 SUMO 交通仿真（了解）

**学什么**：SUMO（Simulation of Urban Mobility）是一个微观交通仿真器，用于模拟大规模交通流。

**为什么学**：在自动驾驶仿真中，SUMO 常与 CARLA 配合使用，SUMO 负责交通流宏观调度，CARLA 负责高保真渲染。部分公司（百度 Apollo）也用 SUMO 做场景测试。

**核心信息**：
- 开源、跨平台
- 支持车辆、行人、交通信号灯仿真
- 可通过 TraCI 接口与 Python 交互
- 与 CARLA 联合仿真：https://carla.readthedocs.io/en/latest/tutorials/G_sumo/

**参考**：
- SUMO 官方文档：https://sumo.dlr.de/docs/

---

## Part 3: 模型部署与优化

### 3.1 PyTorch → ONNX 导出

**学什么**：将 PyTorch 模型导出为 ONNX 格式，作为模型部署的第一步。

**为什么学**：ONNX 是模型部署的通用中间格式。自动驾驶公司普遍使用 ONNX 作为模型训练和部署之间的桥梁。

**学到什么程度**：能正确导出检测、分割、分类等常见模型，处理动态 shape、自定义算子等问题。

**学习时间**：2 天

**基本导出**：
```python
import torch
import onnx

# 加载模型
model = MyModel()
model.eval()

# 创建dummy输入
dummy_input = torch.randn(1, 3, 640, 640)

# 导出
torch.onnx.export(
    model,
    dummy_input,
    "model.onnx",
    export_params=True,
    opset_version=17,
    do_constant_folding=True,
    input_names=['input'],
    output_names=['output'],
    dynamic_axes={
        'input': {0: 'batch_size'},
        'output': {0: 'batch_size'}
    }
)

# 验证
onnx_model = onnx.load("model.onnx")
onnx.checker.check_model(onnx_model)
```

**YOLOv8 导出**：
```python
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
model.export(format='onnx', imgsz=640, dynamic=True, simplify=True)
```

**常见问题**：
- 自定义算子不支持：需要写 ONNX 自定义算子或用标准算子替代
- 动态 shape：batch_size、图像尺寸需要设置 dynamic_axes
- 精度损失：检查 opset_version 是否够新

**推荐资源**：
- ONNX 官方文档：https://onnx.ai/onnx/
- PyTorch ONNX 导出教程：https://pytorch.org/docs/stable/onnx.html
- onnx-simplifier（简化 ONNX 图）：https://github.com/daquexian/onnx-simplifier

**检验标准**：能将 YOLOv8 导出为 ONNX，用 onnxruntime 推理验证精度一致。

---

### 3.2 ONNX Runtime 推理

**学什么**：使用 ONNX Runtime 进行模型推理。

**为什么学**：ONNX Runtime 是微软开源的推理引擎，支持 CPU/GPU/NPU 多平台，延迟低，适合边缘设备部署。

**学到什么程度**：能用 C++ 和 Python 进行 ONNX Runtime 推理，配置 GPU 加速和线程数。

**学习时间**：1-2 天

**Python 推理**：
```python
import onnxruntime as ort
import numpy as np

# 创建推理会话
providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
session = ort.InferenceSession("model.onnx", providers=providers)

# 推理
input_data = np.random.randn(1, 3, 640, 640).astype(np.float32)
outputs = session.run(None, {'input': input_data})
```

**C++ 推理**：
```cpp
#include <onnxruntime_cxx_api.h>

Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "my_app");
Ort::SessionOptions session_options;
session_options.SetIntraOpNumThreads(4);
session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

// GPU加速
OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0);

Ort::Session session(env, "model.onnx", session_options);

// 推理
auto memory_info = Ort::MemoryInfo::CreateCpuOrt(
    OrtArenaAllocator, OrtMemTypeDefault);
std::vector<float> input_tensor_values(batch_size * 3 * 640 * 640);
std::vector<int64_t> input_shape = {batch_size, 3, 640, 640};

Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    memory_info, input_tensor_values.data(), input_tensor_values.size(),
    input_shape.data(), input_shape.size());

auto output_tensors = session.Run(Ort::RunOptions{nullptr},
    input_names.data(), &input_tensor, 1,
    output_names.data(), 1);
```

**安装**：
```bash
# Python
pip install onnxruntime-gpu

# C++ (Ubuntu)
# 下载预编译包或从源码编译
# https://onnxruntime.ai/docs/build/
```

**推荐资源**：
- ONNX Runtime 官方文档：https://onnxruntime.ai/
- ONNX Runtime C++ API：https://onnxruntime.ai/docs/api/c/

**检验标准**：能用 C++ + ONNX Runtime 部署 YOLOv8，实现实时推理（>30 FPS on GPU）。

---

### 3.3 TensorRT 部署

**学什么**：NVIDIA TensorRT 是 NVIDIA GPU 上最高性能的推理引擎，自动驾驶公司（小鹏、蔚来、百度等）的生产环境几乎都用 TensorRT。

**为什么学**：TensorRT 部署是自动驾驶工程化的核心技能。面试必考，项目必用。

**学到什么程度**：能将 ONNX 模型转换为 TensorRT Engine，使用 FP16/INT8 量化，用 C++ API 进行推理。

**学习时间**：1-2 周

#### 3.3.1 安装配置

```bash
# 方式1：通过 NVIDIA 官方仓库
sudo apt-get install tensorrt

# 方式2：通过 pip（仅Python API）
pip install tensorrt

# 方式3：通过 Docker（推荐）
docker pull nvcr.io/nvidia/tensorrt:24.01-py3

# 验证安装
python -c "import tensorrt; print(tensorrt.__version__)"
```

#### 3.3.2 Engine 构建

**trtexec 命令行工具**：
```bash
# FP32精度
trtexec --onnx=model.onnx --saveEngine=model_fp32.engine

# FP16精度（推荐，精度损失极小，速度提升2-4倍）
trtexec --onnx=model.onnx --saveEngine=model_fp16.engine --fp16

# INT8精度（需要校准数据集）
trtexec --onnx=model.onnx --saveEngine=model_int8.engine --int8 --calib=calibration_cache.bin

# 动态shape
trtexec --onnx=model.onnx --saveEngine=model.engine --fp16 \
    --minShapes=input:1x3x640x640 \
    --optShapes=input:4x3x640x640 \
    --maxShapes=input:8x3x640x640
```

#### 3.3.3 Python API 推理

```python
import tensorrt as trt
import pycuda.driver as cuda
import numpy as np

# 加载Engine
logger = trt.Logger(trt.Logger.WARNING)
with open("model.engine", "rb") as f:
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(f.read())

context = engine.create_execution_context()

# 分配GPU内存
input_data = np.random.randn(1, 3, 640, 640).astype(np.float32)
d_input = cuda.mem_alloc(input_data.nbytes)
d_output = cuda.mem_alloc(output_size * 4)

# 创建CUDA流
stream = cuda.Stream()

# 推理
cuda.memcpy_htod_async(d_input, input_data, stream)
context.execute_async_v2(bindings=[int(d_input), int(d_output)], stream_handle=stream.handle)
cuda.memcpy_dtoh_async(output, d_output, stream)
stream.synchronize()
```

#### 3.3.4 C++ API 推理（生产环境推荐）

```cpp
#include "NvInfer.h"
#include "NvOnnxParser.h"
#include <cuda_runtime_api.h>

// 构建Engine
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;
    }
} gLogger;

// 解析ONNX
auto builder = nvinfer1::createInferBuilder(gLogger);
auto network = builder->createNetworkV2(1U << static_cast<uint32_t>(
    nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));
auto parser = nvonnxparser::createParser(*network, gLogger);
parser->parseFromFile("model.onnx", static_cast<int>(nvinfer1::ILogger::Severity::kWARNING));

// 配置
auto config = builder->createBuilderConfig();
config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1U << 30);
config->setFlag(nvinfer1::BuilderFlag::kFP16);

// 构建
auto engine = std::unique_ptr<nvinfer1::ICudaEngine>(
    builder->buildSerializedNetwork(*network, *config));

// 反序列化
auto runtime = nvinfer1::createInferRuntime(gLogger);
auto engine_runtime = runtime->deserializeCudaEngine(engine->data(), engine->size());
auto context = engine_runtime->createExecutionContext();

// 推理
void* buffers[2];
cudaMalloc(&buffers[0], input_size * sizeof(float));
cudaMalloc(&buffers[1], output_size * sizeof(float));

cudaStream_t stream;
cudaStreamCreate(&stream);

context->enqueueV2(buffers, stream, nullptr);
cudaStreamSynchronize(stream);
```

#### 3.3.5 自定义 Plugin

**什么时候需要**：当模型中有 TensorRT 不原生支持的算子时。

```cpp
#include "NvInferPlugin.h"

class MyCustomPlugin : public nvinfer1::IPluginV2DynamicExt {
    // 实现必要的虚函数
    int initialize() noexcept override;
    void terminate() noexcept override;
    nvinfer1::DimsExprs getOutputDimensions(
        int outputIndex, const nvinfer1::DimsExprs* inputs,
        int nbInputs, nvinfer1::IExprBuilder& exprBuilder) noexcept override;
    int enqueue(const nvinfer1::PluginTensorDesc* inputDesc,
                const nvinfer1::PluginTensorDesc* outputDesc,
                const void* const* inputs, void* const* outputs,
                void* workspace, cudaStream_t stream) noexcept override;
    // ...
};
```

**推荐资源**：
- TensorRT 官方文档：https://docs.nvidia.com/deeplearning/tensorrt/
- TensorRT GitHub 示例：https://github.com/NVIDIA/TensorRT
- NVIDIA DLI 课程（免费）：https://www.nvidia.com/en-us/training/
- 博客：TensorRT 部署 YOLOv8 完整教程（搜索中文博客）

**检验标准**：能将 YOLOv8 转换为 TensorRT FP16 Engine，C++ 推理达到实时（>60 FPS on RTX 3060），精度损失 <1%。

---

### 3.4 OpenVINO 部署（Intel 平台）

**学什么**：Intel 的推理优化工具，针对 Intel CPU/GPU/VPU 优化。

**为什么学**：部分自动驾驶公司使用 Intel 平台（如 Mobileye），OpenVINO 在 CPU 上的推理性能优于 ONNX Runtime。了解即可，不需要深入。

**学习时间**：2-3 天

**基本用法**：
```python
from openvino.runtime import Core

core = Core()
model = core.read_model("model.onnx")
compiled_model = core.compile_model(model, "CPU")  # 或 "GPU"

infer_request = compiled_model.create_infer_request()
result = infer_request.infer({"input": input_data})
```

**推荐资源**：
- OpenVINO 官方文档：https://docs.openvino.ai/
- OpenVINO GitHub：https://github.com/openvinotoolkit/openvino

**检验标准**：能在 Intel CPU 上用 OpenVINO 部署一个分类模型，了解 INT8 量化流程。

---

### 3.5 NVIDIA Jetson 平台

**学什么**：NVIDIA Jetson 嵌入式平台的开发，这是自动驾驶感知模块实际部署的主流平台。

**为什么学**：自动驾驶公司的感知模块通常部署在 Jetson Orin NX 或类似嵌入式平台上。了解嵌入式部署是工程化能力的重要体现。

**学到什么程度**：能在 Jetson 上部署 TensorRT 模型，理解功耗与性能的权衡。

**学习时间**：1-2 周

#### 3.5.1 平台选择

| 平台 | GPU | CUDA Core | AI算力 | 功耗 | 价格 | 推荐度 |
|------|-----|-----------|--------|------|------|--------|
| Jetson Orin Nano | 1024 | Ampere | 40 TOPS | 7-15W | ~3000 | 入门 |
| Jetson Orin NX 16GB | 1024 | Ampere | 100 TOPS | 10-25W | ~5000 | **强烈推荐** |
| Jetson AGX Orin 64GB | 2048 | Ampere | 275 TOPS | 15-60W | ~15000 | 高端 |

#### 3.5.2 JetPack 安装

```bash
# JetPack 6.x 包含：
# - L4T (Linux for Tegra)
# - CUDA 12.x
# - cuDNN 8.x
# - TensorRT 8.6+
# - OpenCV
# - Vulkan

# 使用 NVIDIA SDK Manager 刷机（需要另一台 x86 主机）
# 或使用 SD 卡镜像（Jetson Orin Nano）

# 验证
jtop  # 安装 sudo pip3 install jetson-stats
nvcc --version
```

#### 3.5.3 TensorRT 在 Jetson 上的优化

```bash
# 在 Jetson 上构建 Engine（必须在目标平台构建，不能跨平台）
trtexec --onnx=model.onnx --saveEngine=model_fp16.engine --fp16

# Jetson 特有优化
# 1. 设置最大功耗模式
sudo nvpmodel -m 0      # MAXN模式
sudo jetson_clocks       # 锁定最大频率

# 2. 使用 DLA（Deep Learning Accelerator）
trtexec --onnx=model.onnx --saveEngine=model_dla.engine --useDLACore=0 --fp16
```

#### 3.5.4 功耗与散热

- Jetson Orin NX 有多种功耗模式（10W/15W/25W）
- 长时间满载需要散热风扇
- 使用 `tegrastats` 监控温度和功耗
- 工业场景需要考虑宽温范围

**推荐资源**：
- NVIDIA Jetson 开发者中心：https://developer.nvidia.com/embedded-computing
- Jetson Zoo（社区资源）：https://elinux.org/Jetson_Zoo
- Jetson AI Lab：https://jetson-ai-lab.com/
- B站搜索 "Jetson Orin 部署" 有大量中文教程

**检验标准**：能在 Jetson Orin NX 上部署 YOLOv8，达到实时推理（>30 FPS），功耗控制在 15W 以内。

---

### 3.6 模型轻量化

**学什么**：通过剪枝、知识蒸馏、量化等技术减小模型体积和推理时间。

**为什么学**：自动驾驶感知模型需要在嵌入式平台上实时运行，模型轻量化是工程化的核心能力。

**学到什么程度**：了解各技术原理，能使用工具进行基本的模型压缩。

**学习时间**：1 周（理论 + 实操）

#### 3.6.1 剪枝

**结构化剪枝**：移除整个卷积核/通道
```python
import torch.nn.utils.prune as prune

# 对卷积层进行L1范数结构化剪枝
prune.ln_structured(module, name='weight', amount=0.3, n=1, dim=0)

# 使剪枝永久化
prune.remove(module, 'weight')
```

**非结构化剪枝**：将个别权重置零
```python
prune.l1_unstructured(module, name='weight', amount=0.3)
```

**推荐工具**：
- PyTorch 原生 prune API
- Torch-Pruning：https://github.com/VainF/Torch-Pruning （强烈推荐）
- NVIDIA Tao Toolkit

#### 3.6.2 知识蒸馏

```python
# 基本蒸馏框架
def distillation_loss(student_logits, teacher_logits, labels, temperature=4.0, alpha=0.7):
    # 软标签损失
    soft_loss = F.kl_div(
        F.log_softmax(student_logits / temperature, dim=1),
        F.softmax(teacher_logits / temperature, dim=1),
        reduction='batchmean'
    ) * (temperature ** 2)
    
    # 硬标签损失
    hard_loss = F.cross_entropy(student_logits, labels)
    
    return alpha * soft_loss + (1 - alpha) * hard_loss
```

#### 3.6.3 量化感知训练（QAT）

```python
import torch.quantization

# 准备QAT
model.qconfig = torch.quantization.get_default_qat_qconfig('fbgemm')
model_prepared = torch.quantization.prepare_qat(model)

# 训练若干epoch
for epoch in range(num_epochs):
    train_one_epoch(model_prepared, ...)

# 转换
model_quantized = torch.quantization.convert(model_prepared)
```

**推荐资源**：
- PyTorch 量化教程：https://pytorch.org/docs/stable/quantization.html
- Torch-Pruning：https://github.com/VainF/Torch-Pruning
- 论文：Learning Efficient Convolutional Networks Through Network Slimming

**检验标准**：能对 YOLOv8 进行结构化剪枝，模型参数减少 30% 后精度下降 <2%。

---

### 3.7 CUDA 编程基础（加分项）

**学什么**：CUDA 并行编程基础，包括核函数编写、内存管理、并行计算模式。

**为什么学**：自动驾驶中大量计算需要 GPU 加速（点云处理、BEV 特征变换、后处理 NMS 等）。掌握 CUDA 编程是高级工程化能力的体现，能让简历脱颖而出。

**学到什么程度**：能编写基本的 CUDA 核函数，理解 GPU 内存层次，能加速简单算法。

**学习时间**：2-3 周

#### 3.7.1 CUDA 核函数基础

```cuda
// 向量加法
__global__ void vectorAdd(const float* a, const float* b, float* c, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        c[idx] = a[idx] + b[idx];
    }
}

int main() {
    int n = 1024 * 1024;
    float *d_a, *d_b, *d_c;
    
    // 分配GPU内存
    cudaMalloc(&d_a, n * sizeof(float));
    cudaMalloc(&d_b, n * sizeof(float));
    cudaMalloc(&d_c, n * sizeof(float));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, h_a, n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, h_b, n * sizeof(float), cudaMemcpyHostToDevice);
    
    // 启动核函数
    int blockSize = 256;
    int numBlocks = (n + blockSize - 1) / blockSize;
    vectorAdd<<<numBlocks, blockSize>>>(d_a, d_b, d_c, n);
    
    // 拷贝结果回CPU
    cudaMemcpy(h_c, d_c, n * sizeof(float), cudaMemcpyDeviceToHost);
    
    // 释放内存
    cudaFree(d_a); cudaFree(d_b); cudaFree(d_c);
    return 0;
}
```

#### 3.7.2 内存层次

| 内存类型 | 大小 | 访问速度 | 作用域 |
|---------|------|---------|--------|
| Global Memory | GB级 | 慢 | 所有线程 |
| Shared Memory | 48-164 KB | 快（~100x Global） | Block内所有线程 |
| Constant Memory | 64 KB | 快（有Cache） | 所有线程（只读） |
| Registers | 数百KB | 最快 | 单个线程 |
| L1/L2 Cache | 自动 | 快 | 自动管理 |

**Shared Memory 示例（矩阵转置）**：
```cuda
__global__ void transpose(const float* input, float* output, int width, int height) {
    __shared__ float tile[32][32 + 1]; // +1避免bank conflict
    
    int x = blockIdx.x * 32 + threadIdx.x;
    int y = blockIdx.y * 32 + threadIdx.y;
    
    if (x < width && y < height)
        tile[threadIdx.y][threadIdx.x] = input[y * width + x];
    
    __syncthreads();
    
    x = blockIdx.y * 32 + threadIdx.x;
    y = blockIdx.x * 32 + threadIdx.y;
    
    if (x < height && y < width)
        output[y * height + x] = tile[threadIdx.x][threadIdx.y];
}
```

#### 3.7.3 点云处理中的 CUDA 加速

**体素化（Voxelization）**：
```cuda
__global__ void voxelize_kernel(const float* points, int* voxel_indices, 
                                 int num_points, float voxel_size,
                                 int grid_x, int grid_y, int grid_z) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;
    
    float x = points[idx * 3 + 0];
    float y = points[idx * 3 + 1];
    float z = points[idx * 3 + 2];
    
    int vx = (int)(x / voxel_size);
    int vy = (int)(y / voxel_size);
    int vz = (int)(z / voxel_size);
    
    if (vx >= 0 && vx < grid_x && vy >= 0 && vy < grid_y && vz >= 0 && vz < grid_z) {
        voxel_indices[idx] = vx * grid_y * grid_z + vy * grid_z + vz;
        atomicAdd(&voxel_point_count[voxel_indices[idx]], 1);
    }
}
```

**推荐资源**：
- 书籍：《CUDA by Example》（入门首选）
- 书籍：《Programming Massively Parallel Processors》（深入）
- NVIDIA CUDA 官方教程：https://developer.nvidia.com/cuda-education
- CUDA C++ Programming Guide：https://docs.nvidia.com/cuda/cuda-c-programming-guide/
- 课程：CMU 15-418 Parallel Computer Architecture and Programming
- GitHub：搜索 "CUDA point cloud" 有大量参考实现

**检验标准**：能用 CUDA 实现一个简单的点云体素化，性能比 Python 快 100 倍以上。能解释 Shared Memory 的 bank conflict 问题。

---

## Part 4: 自动驾驶开源框架

### 4.1 Apollo（百度）

**学什么**：百度 Apollo 自动驾驶平台的架构和核心模块。

**为什么学**：Apollo 是国内最成熟的自动驾驶开源平台，百度内部和众多合作伙伴使用。了解 Apollo 架构有助于理解自动驾驶系统的工程实践。

**学到什么程度**：理解 Apollo 的整体架构、各模块功能，能在 Dreamview 中运行仿真。不需要深入源码。

**学习时间**：1 周

#### 4.1.1 架构概述

Apollo 的软件架构分为以下层次：

```
┌─────────────────────────────────────────────┐
│                Cloud Service                │
│  (HD Map, Simulation, Data Platform, V2X)   │
├─────────────────────────────────────────────┤
│              Application Layer              │
│  (Routing, Navigation, Teleop)              │
├─────────────────────────────────────────────┤
│          Apollo 9.0 Open Software           │
│  ┌──────────┬──────────┬──────────┐        │
│  │ Perception│Planning  │ Control  │        │
│  ├──────────┼──────────┼──────────┤        │
│  │Prediction│Localization│Guardian│        │
│  └──────────┴──────────┴──────────┘        │
├─────────────────────────────────────────────┤
│              CyberRT Middleware              │
├─────────────────────────────────────────────┤
│              Hardware Abstraction            │
│  (Camera, LiDAR, Radar, GPS, IMU, CAN)      │
└─────────────────────────────────────────────┘
```

#### 4.1.2 感知模块

- **2D 检测**：基于 Camera 的交通灯、车道线检测
- **3D 检测**：基于 LiDAR 的 PointPillars、CenterPoint
- **多传感器融合**：Camera + LiDAR + Radar 融合感知
- **关键特点**：大量使用 TensorRT 加速

#### 4.1.3 预测模块

- **意图预测**：其他车辆的转向/变道意图
- **轨迹预测**：多模态轨迹预测（多条可能的未来轨迹）
- 基于 RNN/Transformer 的预测模型

#### 4.1.4 规划模块

- **路线规划（Routing）**：全局路径规划
- **行为规划**：决策（跟车、变道、停车等）
- **轨迹规划**：Lattice Planner、EM Planner
- **关键特点**：使用 Frenet 坐标系

#### 4.1.5 控制模块

- **纵向控制**：PID / LQR 速度控制
- **横向控制**：LQR / MPC 方向控制
- **关键特点**：支持不同控制策略切换

#### 4.1.6 Dreamview 可视化

Apollo 的 Web 端可视化工具，类似 rviz2 但更专业：
- 实时显示感知、规划、控制结果
- 支持场景回放
- 支持仿真调试

#### 4.1.7 CyberRT 中间件

Apollo 自研的通信中间件（替代 ROS）：
- 共享内存通信（零拷贝）
- 协程调度
- 组件化架构
- 性能优于 ROS1，但生态不如 ROS2

**推荐资源**：
- Apollo 官方文档：https://apollo.baidu.com/
- Apollo GitHub：https://github.com/ApolloAuto/apollo
- Apollo Studio 在线仿真：https://studio.apollo.auto/
- B站搜索 "Apollo 自动驾驶教程"

**检验标准**：能在 Apollo Dreamview 中运行仿真 demo，理解感知→预测→规划→控制的数据流。

---

### 4.2 Autoware（开源自动驾驶全栈）

**学什么**：Autoware 是基于 ROS2 的开源自动驾驶全栈框架，覆盖感知、定位、规划、控制全链路。

**为什么学**：Autoware 是你最应该深入学习的框架。原因：(1) 基于 ROS2，与你的技术栈完美契合；(2) 架构清晰，适合学习自动驾驶系统全貌；(3) 被众多公司作为参考框架。

**学到什么程度**：能在仿真环境中运行 Autoware，理解各模块接口，能修改和扩展功能。

**学习时间**：2-3 周

#### 4.2.1 架构概述

Autoware.Universe（最新版本）的架构：

```
┌─────────────────────────────────────────────┐
│                UI / Visualization            │
│          (rviz2, Web UI)                     │
├─────────────────────────────────────────────┤
│                 Planning                     │
│  Mission Planning → Behavior Planning →      │
│  Motion Planning (Lattice/Bezier/Optimization)│
├─────────────────────────────────────────────┤
│                 Perception                   │
│  Detection (LiDAR/Camera/Fusion) →           │
│  Prediction (Behavior/Trajectory) →           │
│  Traffic Light Recognition                    │
├─────────────────────────────────────────────┤
│                 Localization                 │
│  NDT Matching / EKF / GNSS + IMU             │
├─────────────────────────────────────────────┤
│                 Map                          │
│  HD Map (Lanelet2 Format)                    │
├─────────────────────────────────────────────┤
│                 Vehicle                      │
│  Vehicle Interface → CAN Driver              │
└─────────────────────────────────────────────┘
```

#### 4.2.2 感知模块

- **LiDAR 检测**：CenterPoint, PointPillars, TransFusion
- **相机检测**：YOLOX, Detic
- **融合**：LiDAR-Camera 融合检测
- **跟踪**：AB3DMOT, SimpleTracker
- **交通灯**：基于 CNN 的交通灯识别

#### 4.2.3 规划模块

- **任务规划（Mission Planning）**：全局路径规划，基于 Lanelet2 地图
- **行为规划（Behavior Planning）**：场景管理（跟车、变道、避障等）
- **运动规划（Motion Planning）**：
  - 路径规划：Lattice Planner, Bezier Planner
  - 速度规划：Optimization-based
  - 后处理：障碍物避让、平滑

#### 4.2.4 与 ROS2 的关系

- 完全基于 ROS2 开发
- 所有模块都是 ROS2 节点
- 使用 Launch 文件编排系统
- 使用 QoS 配置通信
- 使用 Lifecycle Node 管理节点状态

**安装 Autoware**：
```bash
# 推荐使用 Docker
docker pull ghcr.io/autowarefoundation/autoware:latest

# 或从源码编译
mkdir -p autoware/src
cd autoware
vcs import src < autoware.repos
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**推荐资源**：
- Autoware 官方文档：https://autoware.org/autoware-documentation/
- Autoware GitHub：https://github.com/autowarefoundation/autoware
- Autoware Universe：https://github.com/autowarefoundation/autoware.universe
- Lanelet2（地图格式）：https://github.com/fzi-forschungszentrum-informatik/Lanelet2
- B站/知乎搜索 "Autoware 教程"

**检验标准**：能在 CARLA 或 AWSIM 仿真环境中运行 Autoware，实现从 A 点到 B 点的自动驾驶，能修改规划器参数并观察行为变化。

---

### 4.3 Apollo vs Autoware 对比

| 维度 | Apollo | Autoware |
|------|--------|----------|
| 中间件 | CyberRT（自研） | ROS2（标准） |
| 生态 | 百度主导 | 社区驱动 |
| 学习曲线 | 较陡（自研框架） | 较平缓（基于ROS2） |
| 代码质量 | 工业级 | 社区级（持续改进中） |
| 仿真 | Dreamview | AWSIM / CARLA |
| 地图 | Apollo HD Map | Lanelet2 |
| 适用场景 | 国内产业 | 国际学术+产业 |
| 社区活跃度 | 高（国内） | 高（国际） |
| 与ROS2兼容性 | 需桥接 | 原生 |

**建议**：以 Autoware 为主（基于 ROS2，学习成本低，技术可迁移），了解 Apollo 架构（拓宽视野，国内公司面试加分）。

---

## Part 5: 必做项目清单

### 项目 1：YOLOv8 目标检测入门（1-2 周）

**项目目标**：掌握深度学习目标检测的基本流程，从数据准备到模型训练再到部署推理。

**为什么做**：目标检测是自动驾驶感知的基础。YOLOv8 是当前最流行的检测模型，生态成熟。这个项目是后续所有感知项目的基础。

**详细步骤**：

**Step 1：环境搭建（0.5 天）**
```bash
pip install ultralytics
pip install onnxruntime-gpu
```

**Step 2：数据集准备（1-2 天）**

方案 A：COCO 子集
```python
from ultralytics import YOLO

# 直接使用COCO预训练权重
model = YOLO('yolov8n.pt')  # nano模型，适合入门
```

方案 B：自定义数据集（推荐，更有区分度）
```python
# 数据集目录结构
# dataset/
#   ├── train/
#   │   ├── images/
#   │   └── labels/
#   ├── val/
#   │   ├── images/
#   │   └── labels/
#   └── data.yaml

# data.yaml
"""
train: ./train/images
val: ./val/images
nc: 3
names: ['ship', 'buoy', 'dock']
"""
```

标注工具推荐：
- **Label Studio**（推荐，Web端）：https://labelstud.io/
- **Roboflow**：https://roboflow.com/
- **CVAT**：https://cvat.ai/
- **LabelImg**：https://github.com/HumanSignal/labelImg

**Step 3：模型训练（1-2 天）**
```python
from ultralytics import YOLO

# 训练
model = YOLO('yolov8s.pt')  # small模型
results = model.train(
    data='dataset/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    device=0,
    patience=20,      # 早停
    workers=8,
    optimizer='AdamW',
    lr0=0.01,
    augment=True,     # 数据增强
)
```

**Step 4：评估（0.5 天）**
```python
model = YOLO('runs/detect/train/weights/best.pt')

# 在验证集上评估
results = model.val()
print(f"mAP@0.5: {results.box.map50:.4f}")
print(f"mAP@0.5:0.95: {results.box.map:.4f}")
print(f"Precision: {results.box.mp:.4f}")
print(f"Recall: {results.box.mr:.4f}")
```

**评估指标解释**：
- **Precision（精确率）**：预测为正的样本中，真正为正的比例
- **Recall（召回率）**：所有真正为正的样本中，被正确预测的比例
- **mAP@0.5**：IoU 阈值为 0.5 时的平均精度
- **mAP@0.5:0.95**：IoU 从 0.5 到 0.95 的平均精度（COCO 标准指标）

**Step 5：ONNX 导出与推理（1 天）**
```python
from ultralytics import YOLO

# 导出
model = YOLO('runs/detect/train/weights/best.pt')
model.export(format='onnx', imgsz=640, dynamic=True, simplify=True)

# ONNX推理验证
import onnxruntime as ort
import cv2
import numpy as np

session = ort.InferenceSession("best.onnx", providers=['CUDAExecutionProvider'])

img = cv2.imread("test.jpg")
img_resized = cv2.resize(img, (640, 640))
input_data = img_resized.transpose(2, 0, 1).astype(np.float32) / 255.0
input_data = np.expand_dims(input_data, 0)

outputs = session.run(None, {'images': input_data})
```

**产出**：
- GitHub 仓库（含 README、训练脚本、推理脚本）
- 训练日志（loss 曲线、mAP 曲线）
- 可视化检测结果图片
- ONNX 模型文件

**推荐资源**：
- Ultralytics 官方文档：https://docs.ultralytics.com/
- YOLOv8 GitHub：https://github.com/ultralytics/ultralytics
- 论文：YOLOv8 官方技术报告

**检验标准**：mAP@0.5 > 0.8（自定义数据集），ONNX 推理延迟 < 10ms（GPU），GitHub 仓库有完整 README。

---

### 项目 2：ROS2 自主导航系统（2-3 周）

**项目目标**：在 Gazebo 中搭建完整的自主导航系统，集成 SLAM 建图、Nav2 导航、自定义感知节点。

**为什么做**：Nav2 是 ROS2 生态中最核心的导航框架，掌握它等于掌握了 ROS2 工程能力的核心。这个项目能充分展示你的 ROS2 + 感知 + 规划综合能力。

**详细步骤**：

**Step 1：Gazebo 仿真环境搭建（2-3 天）**
```bash
# 安装依赖
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-robot-localization
```

创建一个包含走廊、房间、障碍物的 Gazebo 世界，包含差速机器人模型（带 LiDAR、IMU）。

**Step 2：SLAM 建图（2-3 天）**
```bash
# 启动仿真环境
ros2 launch my_robot_gazebo robot_world.launch.py

# 启动 SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# 用键盘控制机器人运动建图
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 保存地图
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: 'my_map'}"
```

**Step 3：Nav2 导航配置（3-5 天）**

nav2_params.yaml 关键配置：
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # ... 参数配置

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavFnPlanner"

behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "wait"]
    # ...

bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "..."
```

启动导航：
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map:=my_map.yaml
```

发送导航目标：
```bash
# 通过rviz2的"2D Nav Goal"按钮
# 或通过代码
ros2 topic pub /goal_pose geometry_msgs/PoseStamped ...
```

**Step 4：自定义检测节点集成（3-5 天）**

将项目 1 的 YOLOv8 检测模型集成到导航系统中，实现：
- 检测到特定目标（如行人）时触发避障行为
- 检测到目标时发布标记在 rviz2 中可视化

```python
class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.subscriber = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(
            MarkerArray, '/detection/markers', 10)
        
        # 加载ONNX模型
        self.session = ort.InferenceSession("yolov8.onnx")
    
    def image_callback(self, msg):
        # 推理
        image = self.bridge.imgmsg_to_cv2(msg)
        results = self.detect(image)
        
        # 发布可视化标记
        markers = self.create_markers(results)
        self.publisher.publish(markers)
```

**Step 5：集成测试与录制（2 天）**

录制完整 demo：机器人从 A 点出发，自主建图，检测障碍物，导航到 B 点。

**产出**：
- 完整 ROS2 包（含 launch 文件、参数文件、节点代码）
- Demo 视频（含建图、导航、检测全过程）
- README 文档（含环境配置、运行步骤）

**推荐资源**：
- Nav2 官方文档：https://navigation.ros.org/
- Nav2 Tutorials：https://navigation.ros.org/tutorials/
- SLAM Toolbox：https://github.com/SteveMacenski/slam_toolbox
- 古月居 Nav2 教程

**检验标准**：机器人能在 Gazebo 中自主导航到指定目标点，途中能识别并避让障碍物，demo 视频流畅。

---

### 项目 3：LiDAR 惯性 SLAM（3-4 周）

**项目目标**：掌握 LiDAR-IMU 融合 SLAM 系统的使用和改进。

**为什么做**：LiDAR SLAM 是自动驾驶定位的核心技术。掌握 FAST-LIO2 或 LIO-SAM 是 SLAM 岗位的基本要求。这个项目能展示你的定位/SLAM 能力。

**详细步骤**：

**Step 1：环境搭建（1 天）**

FAST-LIO2：
```bash
cd ~/ros2_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init

# 安装依赖
pip install pyquaternion

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

LIO-SAM：
```bash
cd ~/ros2_ws/src
git clone https://github.com/TixiaoShan/LIO-SAM.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

**Step 2：在公开数据集上跑通（3-5 天）**

推荐数据集：
- **KITTI**：https://www.cvlibs.net/datasets/kitti/
- **MulRan**：https://github.com/url-kaist/MulRan-Dataset
- **Newer College Dataset**：https://ori-drs.github.io/newer-college-dataset/
- **Hilti SLAM Challenge**：https://hilti-challenge.com/

```bash
# 下载数据集
# 以KITTI为例，需要将原始数据转为rosbag

# 运行FAST-LIO2
ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml

# 播放数据
ros2 bag play kitti_sequence_00.bag
```

**Step 3：改进（1-2 周）**

方向 A：加入回环检测
- 集成 ScanContext 或 ICP 回环检测
- 基于 GPS 的回环约束
- 使用 pose graph 优化

方向 B：自定义特征提取
- 修改点云预处理（地面分割、聚类）
- 自定义特征点提取策略
- 针对特定场景（如海洋环境）的优化

方向 C：多传感器融合
- 加入 GPS 约束
- 加入视觉信息（VIO）
- 动态物体过滤

**Step 4：实验评估（3-5 天）**

使用 **evo** 工具进行轨迹评估：
```bash
# 安装 evo
pip install evo

# 评估ATE（绝对轨迹误差）
evo_ape kitti gt.txt estimated.txt -r full --plot --plot_mode xyz

# 评估RPE（相对位姿误差）
evo_rpe kitti gt.txt estimated.txt -r trans_deg --plot

# 可视化
evo_traj kitti gt.txt estimated.txt --ref=gt.txt -p --plot_mode xyz
```

指标解释：
- **ATE（Absolute Trajectory Error）**：全局轨迹误差，反映累积漂移
- **RPE（Relative Pose Error）**：相对位姿误差，反映局部精度

**产出**：
- 完整的实验代码和配置文件
- 实验对比表格（不同数据集、不同配置的 ATE/RPE）
- 轨迹可视化图
- 改进前后的对比分析

**推荐资源**：
- FAST-LIO2 论文：https://arxiv.org/abs/2107.00862
- FAST-LIO2 GitHub：https://github.com/hku-mars/FAST_LIO
- LIO-SAM 论文：https://arxiv.org/abs/2007.00587
- LIO-SAM GitHub：https://github.com/TixiaoShan/LIO-SAM
- evo 评估工具：https://github.com/MichaelGrupp/evo
- 高翔《视觉SLAM十四讲》（理论基础）

**检验标准**：在 KITTI 数据集上 ATE < 5m（100m 序列），轨迹可视化与真值基本吻合。

---

### 项目 4：USV/UUV 路径规划系统（2-3 周）

**项目目标**：实现多种路径规划和跟踪算法的对比，展示你的规划控制能力。这个项目充分利用你的海洋背景。

**为什么做**：路径规划是自动驾驶的核心模块。这个项目与你的导师方向（云洲智能合作）相关，同时技术完全通用。在面试中，你能用"USV 自主路径规划"来展示规划能力，而不局限于海洋场景。

**详细步骤**：

**Step 1：实现规划算法（1 周）**

A* 算法：
```python
import heapq

def astar(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            return reconstruct_path(came_from, current)
        
        for neighbor in get_neighbors(grid, current):
            tentative_g = g_score[current] + distance(current, neighbor)
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
    
    return None  # 无路径
```

RRT* 算法：
```python
import numpy as np

class RRTStar:
    def __init__(self, start, goal, obstacle_list, bounds):
        self.start = start
        self.goal = goal
        self.obstacle_list = obstacle_list
        self.bounds = bounds
        self.node_list = [start]
    
    def planning(self, max_iter=1000):
        for i in range(max_iter):
            # 随机采样
            rnd = self.sample()
            # 找最近节点
            nearest = self.get_nearest(rnd)
            # 扩展
            new_node = self.steer(nearest, rnd)
            # 碰撞检查
            if not self.collision_check(new_node):
                continue
            # 选择父节点（RRT*特有）
            near_nodes = self.find_near_nodes(new_node)
            new_node = self.choose_parent(new_node, near_nodes)
            self.node_list.append(new_node)
            # 重布线（RRT*特有）
            self.rewire(new_node, near_nodes)
            # 检查是否到达目标
            if self.reached_goal(new_node):
                return self.generate_path(new_node)
        return None
```

DWA 局部规划：
```python
class DWAPlanner:
    def __init__(self, config):
        self.max_speed = config['max_speed']
        self.min_speed = config['min_speed']
        self.max_yaw_rate = config['max_yaw_rate']
        self.dt = config['dt']
        self.predict_time = config['predict_time']
    
    def planning(self, state, goal, obstacles):
        best_u = [0.0, 0.0]
        best_score = -float('inf')
        
        # 搜索速度空间
        for v in np.arange(self.min_speed, self.max_speed, 0.1):
            for w in np.arange(-self.max_yaw_rate, self.max_yaw_rate, 0.1):
                # 轨迹预测
                trajectory = self.predict_trajectory(state, v, w)
                # 评估
                score = self.evaluate(trajectory, goal, obstacles)
                if score > best_score:
                    best_score = score
                    best_u = [v, w]
        
        return best_u
```

**Step 2：MPC 路径跟踪控制器（3-5 天）**
```python
import numpy as np
from scipy.optimize import minimize

class MPCController:
    def __init__(self, N=20, dt=0.1):
        self.N = N  # 预测时域
        self.dt = dt
        self.Q = np.diag([10, 10, 5])  # 状态权重
        self.R = np.diag([1, 1])         # 控制权重
    
    def compute_control(self, state, reference_trajectory):
        # 定义优化问题
        u0 = np.zeros(2 * self.N)
        
        result = minimize(
            self.cost_function,
            u0,
            args=(state, reference_trajectory),
            method='SLSQP',
            bounds=self.get_bounds()
        )
        
        return result.x[:2]  # 返回第一步控制
    
    def cost_function(self, u, state, ref):
        cost = 0
        x = state.copy()
        for i in range(self.N):
            u_i = u[2*i:2*i+2]
            x = self.dynamics(x, u_i)
            ref_i = ref[i]
            cost += (x - ref_i) @ self.Q @ (x - ref_i)
            cost += u_i @ self.R @ u_i
        return cost
```

**Step 3：动态障碍物避障（2-3 天）**
- 实现障碍物检测和跟踪
- 集成 VFH（Vector Field Histogram）避障
- 与 Nav2 的行为树（Behavior Tree）集成

**Step 4：ROS2 集成与可视化（2-3 天）**
- 将算法封装为 ROS2 节点
- 在 rviz2 中可视化规划路径、障碍物、机器人轨迹
- 与 Nav2 集成

**Step 5：对比分析（1-2 天）**

| 算法 | 全局/局部 | 最优性 | 实时性 | 适用场景 |
|------|----------|--------|--------|---------|
| A* | 全局 | 最优 | 一般 | 栅格地图 |
| RRT* | 全局 | 渐近最优 | 较慢 | 高维空间 |
| DWA | 局部 | 局部最优 | 快 | 实时避障 |
| MPC | 局部 | 优化 | 中等 | 约束控制 |

**产出**：
- 4 种以上算法的 ROS2 实现
- 对比视频（相同环境下不同算法的表现）
- 性能对比表（路径长度、规划时间、安全性）
- 完整 ROS2 包

**推荐资源**：
- 书籍：《Planning Algorithms》（Steven LaValle）：http://planning.cs.uiuc.edu/
- 书籍：《Probabilistic Robotics》（Thrun）相关章节
- Nav2 文档：https://navigation.ros.org/
- PythonRobotics：https://github.com/AtsushiSakai/PythonRobotics （强烈推荐，大量算法Python实现）

**检验标准**：4 种算法在仿真环境中都能正常运行，有完整的性能对比报告。

---

### 项目 5：3D 目标检测实战（3-4 周）

**项目目标**：掌握 3D 目标检测的训练和部署全流程。

**为什么做**：3D 目标检测是自动驾驶感知的核心。几乎所有感知岗位都要求掌握 3D 检测。这个项目直接对标面试和工作需求。

**详细步骤**：

**Step 1：mmdetection3d 环境搭建（1-2 天）**
```bash
# 创建conda环境
conda create -n mmdet3d python=3.8
conda activate mmdet3d

# 安装PyTorch
pip install torch==1.13.1+cu117 torchvision==0.14.1+cu117 --extra-index-url https://download.pytorch.org/whl/cu117

# 安装mm系列
pip install -U openmim
mim install mmengine
mim install mmcv==2.0.0
mim install mmdet==3.0.0
mim install mmdet3d==1.1.0
```

**Step 2：数据集准备（2-3 天）**

KITTI 数据集：
- 下载：https://www.cvlibs.net/datasets/kitti/
- 数据格式转换
- 点云预处理

nuScenes 数据集：
- 下载：https://www.nuscenes.org/
- 安装 nuscenes-devkit
- 数据格式理解

**Step 3：训练模型（1 周）**

PointPillars（推荐先跑通这个）：
```bash
# 训练
python tools/train.py configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py

# 评估
python tools/test.py configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py \
    work_dirs/pointpillars/latest.pth --eval mAP
```

CenterPoint（进阶）：
```bash
python tools/train.py configs/centerpoint/centerpoint_voxel01_second_secfpn_head-circlenms_8xb4-cyclic-20e_nus-3d.py
```

**Step 4：TensorRT 部署（1 周）**

mmdet3d 提供了部署工具：
```bash
# 导出ONNX
python tools/deploy.py \
    configs/pointpillars/pointpillars_trt.py \
    configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py \
    work_dirs/pointpillars/latest.pth \
    demo/data/pcd/kitti.pcd \
    --work-dir work_dirs/deploy \
    --device cuda:0
```

也可以手动导出 ONNX 并用 TensorRT 构建 Engine。

**Step 5：海洋场景适配（可选，3-5 天）**

如果有海洋场景的点云数据：
- 自定义数据集格式适配 mmdet3d
- 修改数据加载器
- Fine-tune 预训练模型

**产出**：
- 训练代码和配置文件
- 在 KITTI/nuScenes 上的评测结果（mAP, NDS）
- TensorRT 部署代码
- 性能对比表（FP32 vs FP16 vs INT8）

**推荐资源**：
- mmdetection3d 文档：https://mmdetection3d.readthedocs.io/
- mmdetection3d GitHub：https://github.com/open-mmlab/mmdetection3d
- KITTI 3D 检测评测：https://www.cvlibs.net/datasets/kitti/eval_object.php
- nuScenes 检测评测：https://www.nuscenes.org/object-detection

**检验标准**：PointPillars 在 KITTI 上 Car Moderate mAP > 70%，TensorRT FP16 推理延迟 < 50ms。

---

### 项目 6：BEV 感知系统（4-6 周，进阶）

**项目目标**：复现 BEV 感知范式，理解多相机/多传感器融合的最新技术。

**为什么做**：BEV（Bird's Eye View）感知是当前自动驾驶感知的主流范式。Tesla、小鹏、蔚来等公司都在使用 BEV 方案。掌握 BEV 感知能让你的简历在众多候选人中脱颖而出。

**详细步骤**：

**Step 1：BEV 原理学习（3-5 天）**
- 理解 BEV 的概念：将多视角图像转换为鸟瞰图表示
- 学习 LSS（Lift, Splat, Shoot）原理
- 学习 BEVFormer 的 Transformer 架构

**Step 2：BEVFormer 复现（2 周）**

```bash
# 基于 mmdetection3d
git clone https://github.com/fundamentalvision/BEVFormer.git
cd BEVFormer

# 数据准备（nuScenes）
python tools/create_data.py nuscenes --root-path ./data/nuscenes --out-dir ./data/nuscenes --extra-tag nuscenes

# 训练
python tools/train.py projects/configs/bevformer/bevformer_base.py
```

**Step 3：BEVFusion 复现（1-2 周）**

BEVFusion 结合了 Camera BEV 和 LiDAR BEV：
```bash
git clone https://github.com/mit-han-lab/bevfusion.git
cd bevfusion
# 训练流程类似
```

**Step 4：评测与分析（3-5 天）**

nuScenes 检测评测指标：
- **mAP**：平均精度
- **ATE**：平均平移误差
- **ASE**：平均尺度误差
- **AOE**：平均朝向误差
- **AVE**：平均速度误差
- **NDS**：nuScenes Detection Score（综合指标）

**产出**：
- 论文级复现报告
- 在 nuScenes 上的评测结果
- 可视化分析（BEV 特征图、检测结果）

**推荐资源**：
- LSS 论文：https://arxiv.org/abs/2008.05711
- BEVFormer 论文：https://arxiv.org/abs/2203.17270
- BEVFusion 论文：https://arxiv.org/abs/2205.13542
- BEVFormer GitHub：https://github.com/fundamentalvision/BEVFormer
- BEVFusion GitHub：https://github.com/mit-han-lab/bevfusion

**检验标准**：能在 nuScenes 上复现 BEVFormer，NDS 接近论文报告值（>50），有详细的实验分析。

---

## Part 6: 实习与求职指南

### 6.1 目标公司与岗位分析

**核心策略**：你有两条最优路径——

**路径 A：字节跳动（利用导师关系）**
- 导师与字节有合作，这是最大的人脉优势
- 字节跳动的自动驾驶团队（原 Pico 相关、机器人方向）待遇优厚
- 提前与导师沟通，争取内推机会
- 准备时间：研一下学期开始了解岗位要求

**路径 B：头部自动驾驶公司（技术路线）**
- 华为、小鹏、蔚来、理想、Momenta 等
- 需要过硬的项目经验和算法能力
- 通过暑期实习转正或校招进入

**目标公司详细分析**：

| 优先级 | 公司 | 岗位方向 | 薪资范围（2029校招预估） | 适合你的原因 |
|--------|------|---------|------------------------|-------------|
| ★★★★★ | 字节跳动 | AI/机器人/具身智能 | 40-60万 | 导师合作关系，最强人脉 |
| ★★★★★ | 华为 | 感知/规划算法（车BU） | 35-55万 | 大量招人，工程能力强优先 |
| ★★★★★ | 小鹏 | 自动驾驶感知/规划 | 30-50万 | 技术导向，重工程能力 |
| ★★★★★ | 蔚来 | 自动驾驶算法 | 30-50万 | 技术导向 |
| ★★★★★ | 理想 | 自动驾驶算法 | 30-50万 | 发展快 |
| ★★★★★ | Momenta | 感知/规划/PNC | 35-60万 | 技术前沿，薪资有竞争力 |
| ★★★★☆ | 百度Apollo | 感知/规划 | 30-45万 | Apollo 生态熟悉加分 |
| ★★★★☆ | 大疆 | 机器人感知/规划 | 30-50万 | 工程能力优先 |
| ★★★★☆ | 宇树科技 | 机器人算法 | 30-50万 | 人形机器人方向 |
| ★★★★☆ | 智元机器人 | 机器人算法 | 30-50万 | 具身智能方向 |
| ★★★★☆ | 地平线 | 感知部署/嵌入式 | 30-50万 | 偏部署，工程能力优先 |
| ★★★★☆ | 黑芝麻智能 | 感知部署/芯片 | 30-50万 | 偏部署 |
| ★★★☆☆ | 云洲智能 | 保底选择 | 20-35万 | 导师合作方，保底 |

**实习时间线建议**：
```
研一上（2025.9-2026.1）：打基础
  - 学习 ROS2、Gazebo、C++ 深入
  - 跑通项目1（YOLOv8）和项目2（Nav2）

研一下（2026.2-2026.6）：积累项目
  - 完成项目3（LiDAR SLAM）和项目4（路径规划）
  - 开始项目5（3D检测）
  - 准备暑期实习投递

研一暑假（2026.7-2026.9）：第一次实习
  - 目标：字节跳动（导师推荐）或自动驾驶公司
  - 感受工业界工作节奏

研二上（2026.9-2027.1）：深入技术
  - 完成项目6（BEV感知）
  - 发表论文（ICRA/IROS 投稿）
  - 准备第二次实习

研二下（2027.2-2027.6）：第二次实习
  - 目标：目标公司的实习（暑期实习或日常实习）
  - 拿return offer

研二暑假（2027.7-2027.9）：秋招
  - 投递秋招，利用两次实习经验
  - 同时准备论文

研三（2027.9-2029.6）：论文+入职准备
```

**说明**：你 2029 年毕业，实际上时间很充裕。按照上述时间线，你有将近 3 年时间准备，完全可以做到非常充分。

---

### 6.2 面试高频题清单

#### 6.2.1 C++ 面试题

**智能指针**
```cpp
// shared_ptr：引用计数共享所有权
auto p1 = std::make_shared<int>(42);
auto p2 = p1;  // 引用计数 +1
// 问题：循环引用导致内存泄漏
// 解决：weak_ptr 打破循环

// unique_ptr：独占所有权，不可拷贝
auto p = std::make_unique<int>(42);
auto p2 = std::move(p);  // 只能移动，不能拷贝

// weak_ptr：不影响引用计数的观察者
std::weak_ptr<int> wp = p1;
if (auto sp = wp.lock()) {
    // sp 有效，使用
}
```

面试常问：
- shared_ptr 的线程安全性？（引用计数操作是线程安全的，但对象读写不是）
- make_shared vs new + shared_ptr 的区别？（make_shared 一次内存分配，性能更好）
- unique_ptr 的使用场景？（工厂函数返回值、容器元素）

**虚函数与多态**
```cpp
class Base {
public:
    virtual void speak() { std::cout << "Base" << std::endl; }
    virtual ~Base() {}  // 虚析构函数，防止内存泄漏
};

class Derived : public Base {
public:
    void speak() override { std::cout << "Derived" << std::endl; }
};

// 虚函数表（vtable）实现机制
// 每个含虚函数的类有一个vtable
// 对象有一个vptr指向vtable
```

面试常问：
- 虚函数的实现原理（vtable + vptr）
- 纯虚函数 vs 虚函数
- 构造函数中调用虚函数的行为
- 虚析构函数的作用

**右值引用与移动语义**
```cpp
// 左值：有名字、可取地址
int a = 10;  // a 是左值

// 右值：临时对象、字面量
int b = a + 5;  // a+5 是右值

// 右值引用：绑定到右值
void process(std::vector<int>&& vec) {
    // 接管资源，避免拷贝
}

// 移动语义
class Buffer {
    int* data;
    size_t size;
public:
    // 移动构造函数
    Buffer(Buffer&& other) noexcept
        : data(other.data), size(other.size) {
        other.data = nullptr;
        other.size = 0;
    }
    
    // 移动赋值运算符
    Buffer& operator=(Buffer&& other) noexcept {
        if (this != &other) {
            delete[] data;
            data = other.data;
            size = other.size;
            other.data = nullptr;
            other.size = 0;
        }
        return *this;
    }
};

// std::move：将左值转为右值引用
Buffer b1;
Buffer b2 = std::move(b1);  // 移动，不拷贝
```

**STL 容器时间复杂度**：

| 容器 | 随机访问 | 插入/删除 | 查找 |
|------|---------|----------|------|
| vector | O(1) | 尾部O(1)，中间O(n) | O(n) |
| deque | O(1) | 头尾O(1)，中间O(n) | O(n) |
| list | O(n) | O(1)（已知位置） | O(n) |
| map/set | - | O(log n) | O(log n) |
| unordered_map/set | - | O(1)平均 | O(1)平均 |
| priority_queue | - | O(log n) | O(n) |

**内存对齐**：
```cpp
struct A {
    char a;    // 1 byte + 3 padding
    int b;     // 4 bytes
    char c;    // 1 byte + 7 padding
    double d;  // 8 bytes
};  // sizeof(A) = 24

struct B {
    double d;  // 8 bytes
    int b;     // 4 bytes
    char a;    // 1 byte
    char c;    // 1 byte + 2 padding
};  // sizeof(B) = 16  // 排列顺序不同，大小不同！
```

**线程安全与锁**：
```cpp
#include <mutex>
#include <shared_mutex>

std::mutex mtx;
std::shared_mutex shared_mtx;

// 独占锁
void write() {
    std::lock_guard<std::mutex> lock(mtx);
    // 写操作
}

// 共享锁（读写锁，适合读多写少）
void read() {
    std::shared_lock<std::shared_mutex> lock(shared_mtx);
    // 读操作
}

void write() {
    std::unique_lock<std::shared_mutex> lock(shared_mtx);
    // 写操作
}
```

**推荐资源**：
- 书籍：《Effective Modern C++》（Scott Meyers，必读）
- 书籍：《C++ Primer》（第5版）
- 书籍：《深度探索C++对象模型》
- LeetCode C++ 题解：https://leetcode.cn/

---

#### 6.2.2 算法面试题（LeetCode 重点）

**推荐刷题路线**（按优先级）：

| 类别 | 重点题 | 数量 | 优先级 |
|------|--------|------|--------|
| 数组/字符串 | 1, 3, 5, 15, 26, 42, 53, 56, 76 | 15-20 | ★★★★★ |
| 链表 | 2, 19, 21, 23, 141, 142, 206, 234 | 10-15 | ★★★★★ |
| 树 | 94, 98, 101, 102, 104, 105, 226, 236 | 15-20 | ★★★★★ |
| 图 | 200, 207, 210, 743, 785 | 10-15 | ★★★★☆ |
| 动态规划 | 5, 32, 42, 62, 64, 70, 121, 198, 300, 322 | 20-30 | ★★★★★ |
| 排序 | 215, 347, 912（手写快排/归并/堆排） | 5-10 | ★★★★★ |
| 二分查找 | 33, 34, 35, 74, 153, 704 | 10 | ★★★★☆ |
| 双指针 | 11, 15, 42, 76, 438, 567 | 10 | ★★★★★ |
| 滑动窗口 | 3, 76, 239, 438, 567 | 5-8 | ★★★★★ |
| 栈/队列 | 20, 21, 23, 155, 239, 394 | 10 | ★★★★☆ |
| 堆 | 215, 295, 347 | 5 | ★★★★☆ |

**刷题平台**：
- LeetCode：https://leetcode.cn/ （国内版）
- 目标：至少刷 200 题，其中 medium 难度 150+
- 每天 1-2 题，坚持到毕业

**推荐资源**：
- 书籍：《剑指Offer》
- 书籍：《LeetCode Hot 100》题解
- 代码随想录：https://programmercarl.com/ （强烈推荐，中文）
- NeetCode：https://neetcode.io/

---

#### 6.2.3 SLAM 面试题

**Q：视觉 SLAM 前端和后端分别做什么？**

前端（Frontend）：
- 特征提取与匹配（ORB、SIFT 等）
- 光流追踪（LK光流）
- 位姿估计（PnP、ICP、对极约束）
- 关键帧选取

后端（Backend）：
- 位姿图优化（Pose Graph Optimization）
- Bundle Adjustment（BA）
- 回环检测与校正
- 全局一致性优化

**Q：对极约束 / EPnP / ICP 原理**

对极约束：
- 两视图几何关系
- $x_2^T F x_1 = 0$（F 为基础矩阵）
- 8 点法求解 F/H 矩阵
- 从 E 矩阵恢复 R, t（4种可能解）

EPnP：
- 4 个控制点参数化 3D 点
- 将 PnP 问题转化为求解控制点坐标
- 高效，O(n) 复杂度

ICP（Iterative Closest Point）：
- 点对点/点对面匹配
- SVD 求解最优 R, t
- 迭代收敛到局部最优
- 变种：Point-to-Plane ICP、Generalized ICP

**Q：图优化原理**

- 将 SLAM 问题建模为图
- 节点：相机位姿、路标点
- 边：观测约束（里程计、观测）
- 优化目标：最小化所有约束的误差之和
- 求解：Gauss-Newton / Levenberg-Marquardt
- 工具：g2o、Ceres Solver、GTSAM

**Q：回环检测方法**

- 词袋模型（BoW/DBoW2）：将图像描述为视觉词袋向量，比较向量相似度
- NetVLAD：基于深度学习的图像检索
- ScanContext（LiDAR）：基于点云的环形描述子
- ICP 验证：回环候选确认

**Q：视觉 SLAM vs LiDAR SLAM**

| 维度 | 视觉 SLAM | LiDAR SLAM |
|------|-----------|------------|
| 传感器成本 | 低 | 高 |
| 精度 | 中等 | 高 |
| 环境适应性 | 光照敏感 | 不受光照影响 |
| 特征丰富度 | 高（纹理） | 低（几何） |
| 漂移 | 较快 | 较慢 |
| 代表系统 | ORB-SLAM3, VINS-Fusion | LOAM, FAST-LIO2 |

**Q：预积分原理**

- IMU 预积分：将高频 IMU 数据预积分为两帧之间的相对运动
- 避免重复积分：当位姿估计更新时，不需要重新积分
- 增量式更新：预积分因子可以直接用于图优化
- 代表：VINS-Mono/Fusion 中的预积分因子

**Q：卡尔曼滤波家族对比**

| 方法 | 线性 | 计算复杂度 | 精度 | 应用 |
|------|------|-----------|------|------|
| KF | 线性系统 | 低 | 最优 | 线性高斯系统 |
| EKF | 非线性（一阶近似） | 中 | 局部最优 | 早期 SLAM |
| UKF | 非线性（sigma点） | 中高 | 更好 | 状态估计 |
| PF | 任意非线性 | 高 | 粒子表示 | 机器人定位 |
| IEKF | 非线性（迭代） | 中高 | 更好 | FAST-LIO2 |

**推荐资源**：
- 书籍：《视觉SLAM十四讲》（高翔，必读）
- 书籍：《State Estimation for Robotics》（Tim Barfoot）
- 课程：深蓝学院 SLAM 系列课程

---

#### 6.2.4 感知面试题

**Q：YOLO 系列演进与核心改进**

| 版本 | 年份 | 核心改进 |
|------|------|---------|
| YOLOv1 | 2016 | 单阶段检测，网格划分 |
| YOLOv2 | 2017 | BatchNorm、Anchor Box、多尺度训练 |
| YOLOv3 | 2018 | FPN多尺度检测、Darknet-53 |
| YOLOv4 | 2020 | CSPNet、SPP、PANet |
| YOLOv5 | 2020 | 自适应Anchor、Focus层 |
| YOLOv8 | 2023 | Anchor-free、C2f模块、解耦头 |
| YOLOv9 | 2024 | GELAN、PGI |

**Q：NMS 原理与改进**

标准 NMS：
1. 按置信度排序所有检测框
2. 选择最高分的框
3. 删除与该框 IoU > 阈值的其他框
4. 重复直到无框剩余

改进方法：
- Soft-NMS：用 IoU 衰减分数而非直接删除
- DIoU-NMS：考虑中心点距离
- Matrix NMS：并行化，更快
- NMS-Free：如 CenterPoint，不需要 NMS 后处理

**Q：Anchor-based vs Anchor-free**

| 维度 | Anchor-based | Anchor-free |
|------|-------------|-------------|
| 代表 | Faster RCNN, YOLOv3 | CenterNet, FCOS, YOLOv8 |
| 优点 | 利用先验，召回率高 | 简洁，无需预设Anchor |
| 缺点 | Anchor超参敏感 | 小目标检测较弱 |
| 趋势 | 逐步被淘汰 | 主流方向 |

**Q：BEV 感知原理**

核心思路：将多视角 2D 图像特征通过相机参数转换到 3D BEV 空间。

关键步骤：
1. 图像特征提取（ResNet/Swin Transformer）
2. 2D→3D 特征转换（LSS/Transformer）
3. BEV 特征聚合（多尺度、时序融合）
4. BEV 空间检测/分割

代表方法：
- BEVFormer：用 Transformer 查询从图像特征中提取 BEV 特征
- BEVFusion：Camera BEV + LiDAR BEV 融合
- LSS：显式深度估计 + 视锥体投影

**Q：3D 检测中体素化作用**

将无序点云转换为规则的 3D 网格（体素），使得 3D 卷积可以应用。

- PointPillars：将点云编码为柱体（2D体素）
- VoxelNet：3D体素化 + 3D卷积
- 体素化的好处：规则化数据结构，GPU 友好
- 体素化的坏处：量化损失、计算量大

**Q：多传感器融合策略**

| 层次 | 方法 | 特点 |
|------|------|------|
| 前融合 | 数据级融合（点云投影到图像） | 信息保留最多 |
| 特征级融合 | BEV 融合、Transformer 融合 | 精度和效率平衡 |
| 后融合 | 检测结果级融合（NMS合并） | 模块化强，精度最低 |

**Q：Transformer 在检测中的应用**

- DETR：端到端检测，去除 NMS 和 Anchor
- Deformable DETR：可变形注意力，收敛更快
- DINO：最强 DETR 变体
- BEVFormer：用空间/时序注意力生成 BEV 特征
- PETR/StreamPETR：3D 位置编码

**推荐资源**：
- 论文：YOLO 系列原始论文
- 论文：DETR、BEVFormer、BEVFusion
- 书籍：《动手学深度学习》（李沐）检测章节

---

#### 6.2.5 规划面试题

**Q：A* 和 RRT 优缺点对比**

| 维度 | A* | RRT/RRT* |
|------|-----|----------|
| 完备性 | 完备（有解必找到） | 概率完备 |
| 最优性 | 最优（启发函数可容许） | RRT渐近最优，RRT非最优 |
| 适用空间 | 低维（2D/3D栅格） | 高维连续空间 |
| 计算复杂度 | O(b^d) | 取决于采样 |
| 实时性 | 高维时差 | 高维时好 |
| 典型应用 | 2D导航、游戏 | 机械臂规划、高维规划 |

**Q：DWA 原理**

Dynamic Window Approach：
1. 速度空间采样：在 (v, w) 空间中离散采样
2. 轨迹前向模拟：每个 (v, w) 生成一条圆弧轨迹
3. 评估函数：综合目标方向、障碍物距离、速度
4. 选择最优 (v, w) 下发

优点：实时性好，考虑动力学约束
缺点：容易陷入局部最优

**Q：Frenet 坐标系规划**

将道路坐标系分解为：
- s（纵轴）：沿参考线的弧长
- d（横轴）：偏离参考线的横向距离

优点：
- 解耦纵向和横向规划
- 天然适合道路场景
- 规划空间更规则

代表：Apollo 的 Lattice Planner

**Q：MPC 原理与应用**

Model Predictive Control：
1. 建立系统动力学模型
2. 在每个时间步求解有限时域优化问题
3. 仅执行第一步控制
4. 滚动优化

优点：
- 能处理约束（输入约束、状态约束）
- 预测能力强
- 适合非线性系统

缺点：
- 计算量大
- 需要精确的动力学模型
- 参数调节困难

**Q：行为规划方法**

- 有限状态机（FSM）：简单、可解释
- 行为树（Behavior Tree）：Nav2 使用，可组合、可复用
- 决策树/规则系统：基于规则的决策
- 基于学习：强化学习、模仿学习

**Q：Lattice Planner 原理**

1. 在 Frenet 坐标系下采样终端状态（位置、速度、加速度）
2. 用五次多项式连接起始状态和终端状态
3. 生成多条候选轨迹
4. 用代价函数评估各轨迹（安全性、舒适性、效率）
5. 选择最优轨迹

**推荐资源**：
- 书籍：《Planning Algorithms》（LaValle）
- 书籍：《Probabilistic Robotics》
- PythonRobotics：https://github.com/AtsushiSakai/PythonRobotics
- Apollo 规划模块文档

---

#### 6.2.6 自动驾驶综合题

**Q：自动驾驶系统架构**

```
感知（Perception）
  ↓
预测（Prediction）
  ↓
决策（Decision / Behavior Planning）
  ↓
规划（Planning / Motion Planning）
  ↓
控制（Control）
  ↓
执行（Actuation - CAN Bus）
```

各模块职责：
- 感知：环境理解（检测、跟踪、语义分割、定位）
- 预测：其他交通参与者的未来行为/轨迹预测
- 决策：选择行为（跟车、变道、停车等）
- 规划：生成具体路径和速度曲线
- 控制：将路径转换为油门/刹车/转向指令

**Q：Corner Case 处理**

常见 Corner Case：
- 遮挡目标（被大车挡住的行人）
- 极端天气（大雨、大雾、强光）
- 施工区域（临时标志、锥桶）
- 异形车辆（三轮车、拖车）
- 非标准道路（无车道线、匝道）
- 传感器失效（相机过曝、LiDAR 降雪噪声）

处理策略：
- 多传感器冗余
- 保守策略（安全停车）
- 大规模仿真测试
- 数据驱动（收集和标注 Corner Case）
- 规则兜底

**Q：功能安全**

- ISO 26262：汽车功能安全标准
- ASIL 等级：A（最低）→ D（最高）
- SOTIF（ISO 21448）：预期功能安全
- 冗余设计：感知冗余、计算冗余、通信冗余
- 故障检测与安全降级（MRC → MRM）

**Q：仿真测试 vs 真实路测**

| 维度 | 仿真测试 | 真实路测 |
|------|---------|---------|
| 成本 | 低 | 高 |
| 安全性 | 绝对安全 | 有风险 |
| 场景覆盖 | 广（可生成任意场景） | 窄（依赖实际遇到） |
| 真实性 | 较低（Sim-to-Real Gap） | 最高 |
| 里程 | 可无限积累 | 受限于时间和车辆 |
| 策略 | 仿真覆盖 99%，路测验证 1% |

---

### 6.3 简历包装策略

**核心原则**：强调通用技术栈，弱化行业特异性。

**项目描述转换**：

| 你的实际项目 | 简历中的描述 | 强调的技术 |
|------------|-------------|-----------|
| USV 自主感知系统 | 无人系统环境感知方案设计与实现 | 多传感器融合、目标检测、实时推理 |
| 水下目标检测 | 复杂场景下的小目标检测算法 | YOLOv8、数据增强、TensorRT 部署 |
| 海洋路径规划 | 约束环境下的自主路径规划系统 | A*/RRT*/DWA/MPC、ROS2 集成 |
| 无人船 SLAM | 动态环境下的 LiDAR-IMU 融合定位 | FAST-LIO2、回环检测、位姿图优化 |
| USV 控制系统 | 无人平台运动控制与轨迹跟踪 | MPC、PID、状态估计 |
| 海上目标跟踪 | 多目标实时跟踪系统 | 多传感器融合、数据关联、卡尔曼滤波 |

**简历结构建议**：

```
姓名 | 联系方式 | GitHub 链接

教育背景
  - 985 硕士，船舶与海洋工程，2025-2029
  - 双非本科，机器人工程
  - GPA：X.X/4.0（如果不错的话写上）

技术栈
  - 编程语言：C++（熟练）、Python（熟练）、CUDA（了解）
  - 框架：ROS2、Autoware、Nav2、PyTorch、TensorRT
  - 工具：Git、Docker、Gazebo、CARLA、Linux

项目经历（3-4个，每个3-5行）
  项目1：无人系统环境感知方案
    - 基于 YOLOv8 的实时目标检测系统，mAP@0.5 达到 0.85
    - 使用 TensorRT FP16 部署，推理延迟 < 10ms
    - 集成到 ROS2 节点中，实现端到端的感知流程
    
  项目2：...
  
实习经历（如有）
  
论文发表（如有）
  
竞赛/获奖（如有）
```

**推荐资源**：
- 牛客网简历模板：https://www.nowcoder.com/
- 知乎搜索 "自动驾驶简历" 有大量参考

---

## Part 7: 论文策略

### 7.1 选题原则

**核心原则**：通用性强，不海洋特化。

好的选题方向：
- 基于 LiDAR 的 3D 目标检测（可应用于任何场景）
- 多传感器融合定位（不局限于海洋环境）
- BEV 感知（当前热点）
- 端到端自动驾驶（前沿方向）
- 约束环境下的路径规划（通用性强）
- 时序点云分割/检测
- 场景流估计（Scene Flow Estimation）

避免的选题：
- 纯海洋场景特化的算法（面试时无法展示通用价值）
- 过于狭窄的工程问题（缺乏学术价值）

**策略**：将海洋场景作为算法的验证场景之一，而非算法的核心限制。

例如：
- 不好的选题："基于 XXX 的水下目标检测"
- 好的选题："面向小目标的轻量化 3D 目标检测算法——在水下/自动驾驶/无人机场景的验证"

### 7.2 目标会议与期刊

**顶级会议（难度高，含金量最高）**：

| 会议 | 全称 | 截稿时间 | 难度 | 方向 |
|------|------|---------|------|------|
| CVPR | Computer Vision and Pattern Recognition | 每年11月 | ★★★★★ | CV |
| ICCV | International Conference on Computer Vision | 每年3月（奇数年） | ★★★★★ | CV |
| ECCV | European Conference on Computer Vision | 每年3月（偶数年） | ★★★★★ | CV |
| NeurIPS | Neural Information Processing Systems | 每年5月 | ★★★★★ | ML |
| ICRA | IEEE International Conference on Robotics and Automation | 每年9月 | ★★★★☆ | 机器人 |
| IROS | IEEE/RSJ International Conference on Intelligent Robots and Systems | 每年3月 | ★★★★☆ | 机器人 |

**次级会议（推荐先投，积累经验）**：

| 会议 | 全称 | 截稿时间 | 难度 | 方向 |
|------|------|---------|------|------|
| IV | IEEE Intelligent Vehicles Symposium | 每年1月 | ★★★☆☆ | 自动驾驶 |
| ITSC | IEEE International Conference on Intelligent Transportation Systems | 每年3月 | ★★★☆☆ | 智能交通 |
| CoRL | Conference on Robot Learning | 每年6月 | ★★★★☆ | 机器人学习 |
| RSS | Robotics: Science and Systems | 每年1月 | ★★★★★ | 机器人 |

**顶级期刊**：

| 期刊 | 全称 | 难度 | 周期 |
|------|------|------|------|
| T-RO | IEEE Transactions on Robotics | ★★★★★ | 6-12月 |
| RA-L | IEEE Robotics and Automation Letters | ★★★★☆ | 3-6月 |
| T-ITS | IEEE Transactions on Intelligent Transportation Systems | ★★★★☆ | 3-6月 |
| T-PAMI | IEEE Transactions on PAMI | ★★★★★ | 6-12月 |

**保底**：
- Ocean Engineering（海洋 SCI，与你的船舶背景直接相关）
- Journal of Marine Science and Engineering
- Applied Ocean Research

### 7.3 投稿策略

**建议路线**：

```
研一下：准备实验，写初稿
  ↓
研一暑假投稿 ICRA/IROS/IV/ITSC（次级会议）
  ↓
如果被拒 → 根据审稿意见修改 → 投下一个会议
如果录用 → 发表！同时准备下一篇
  ↓
研二上：扩展工作，投 RA-L 或 T-ITS
  ↓
研二下：保底投 Ocean Engineering（与导师方向结合）
```

**写作建议**：

1. **先投会议练手**：会议审稿快（2-3个月），反馈及时，适合积累经验
2. **写作风格**：多读目标会议的论文，模仿其写作结构和语言
3. **图表质量**：用 draw.io、PPT 或 LaTeX/TikZ 画图，不要用 Word
4. **实验充分**：消融实验（Ablation Study）是必须的
5. **代码开源**：增加论文被引用的机会
6. **与导师方向结合**：利用海洋场景数据作为实验场景之一

**推荐资源**：
- 书籍：《学术论文写作》相关书籍
- 工具：LaTeX 模板（Overleaf：https://www.overleaf.com/）
- 网站：Paper with Code（https://paperswithcode.com/）查找 SOTA 方法
- 网站：Semantic Scholar（https://www.semanticscholar.org/）文献检索
- 网站：Connected Papers（https://www.connectedpapers.com/）论文关系图

---

## 附录：学习资源汇总

### 书籍清单（按优先级排序）

| 优先级 | 书名 | 方向 |
|--------|------|------|
| ★★★★★ | 《C++ Primer》（第5版） | C++基础 |
| ★★★★★ | 《Effective Modern C++》 | C++进阶 |
| ★★★★★ | 《视觉SLAM十四讲》（高翔） | SLAM |
| ★★★★★ | 《ROS2智能机器人开发实践》（胡春旭） | ROS2 |
| ★★★★☆ | 《深度学习》（花书） | 深度学习理论 |
| ★★★★☆ | 《动手学深度学习》（李沐） | 深度学习实践 |
| ★★★★☆ | 《Probabilistic Robotics》（Thrun） | 机器人学 |
| ★★★☆☆ | 《Planning Algorithms》（LaValle） | 规划 |
| ★★★☆☆ | 《CUDA by Example》 | CUDA |
| ★★★☆☆ | 《State Estimation for Robotics》（Barfoot） | 状态估计 |

### 在线课程

| 课程 | 平台 | 方向 |
|------|------|------|
| CS231n | Stanford / B站 | 计算机视觉 |
| 深蓝学院 SLAM 系列 | 深蓝学院 | SLAM |
| CS285 Deep RL | UC Berkeley | 强化学习 |
| 古月居 ROS2 教程 | B站 | ROS2 |
| 鱼香ROS 教程 | B站/网站 | ROS2 |
| NVIDIA DLI 课程 | NVIDIA | CUDA / TensorRT |

### GitHub 必 Star 仓库

| 仓库 | 用途 |
|------|------|
| ultralytics/ultralytics | YOLOv8 |
| open-mmlab/mmdetection3d | 3D检测 |
| autowarefoundation/autoware | 自动驾驶全栈 |
| ros-planning/navigation2 | Nav2导航 |
| hku-mars/FAST_LIO | LiDAR SLAM |
| TixiaoShan/LIO-SAM | LiDAR SLAM |
| AtsushiSakai/PythonRobotics | 规划算法 |
| ApolloAuto/apollo | Apollo |
| NVIDIA/TensorRT | 部署 |
| MichaelGrupp/evo | SLAM评估 |

### 刷题平台

| 平台 | 用途 |
|------|------|
| LeetCode（leetcode.cn） | 算法刷题 |
| 牛客网 | 面试题库、面经 |
| 代码随想录 | 系统刷题指南 |

---

## 总结：你的优势与策略

**你的独特优势**：
1. 导师与字节跳动有合作 → 最强人脉优势
2. 云洲智能合作 → 真实项目经验（无人系统）
3. 机器人工程本科 → 有ROS/嵌入式基础
4. 985硕士 → 学历门槛无问题
5. 2029年毕业 → 有充足准备时间（3年）

**核心策略**：
1. 技术通用化：所有项目强调通用技术，不局限于海洋
2. 两线并行：字节（人脉线）+ 自动驾驶公司（技术线）
3. 论文保底：至少 1 篇会议/期刊，保底 Ocean Engineering
4. 实习先行：争取两次实习经历，至少一次在目标公司
5. 持续刷题：LeetCode 200+ 题是校招基本门槛

---

# 模块七：端到端自动驾驶（End-to-End Autonomous Driving）

## 端到端自动驾驶（End-to-End Autonomous Driving）学习指南

> 面向具有机器人工程背景、即将攻读硕士的工程落地型学生

---

## Part 1: 端到端自动驾驶概述

### 1.1 什么是端到端自动驾驶

**学什么**：端到端自动驾驶的核心思想是用一个统一的神经网络模型，以原始传感器数据（摄像头图像、LiDAR点云等）作为输入，直接输出车辆的规划轨迹或控制信号（方向盘角度、油门、刹车），跳过传统方案中人工划分的多个独立模块。

**为什么学**：理解"端到端"这个概念是进入整个领域的前提。你需要搞清楚它与传统方案的本质差异，才能在后续阅读论文时理解每篇工作的设计动机。

**学到什么程度**：能够清晰画出传统模块化架构和端到端架构的流程对比图，能用自己的话解释为什么端到端可以避免"级联误差"（cascading error）问题——即模块间接口信息损失导致的误差累积。

**推荐资源**：
- 论文：Bojarski et al., "End to End Learning for Self-Driving Cars" (arXiv 2016) —— 这是端到端驾驶的开山之作，NVIDIA的DAVE-2系统，从摄像头图像直接输出方向盘转角，务必精读
- 论文：Codevilla et al., "Exploring the Limitations of Behavior Cloning for Autonomous Driving" (ICLR 2019) —— 该文系统分析了行为克隆的局限性，是理解端到端方法挑战的重要参考
- 博客：知乎专栏《端到端自动驾驶：从概念到落地》系列文章，搜索"端到端自动驾驶"可找到多篇高质量中文综述
- B站：赵虚左老师（北京理工大学）的自动驾驶相关课程，讲解模块化与端到端的对比

**检验标准**：能够画出传统方案（感知-预测-规划-控制四模块流水线）和端到端方案的完整数据流图，并清晰说明前者的级联误差问题具体在哪里产生、如何传播。

### 1.2 与传统模块化架构的区别

**学什么**：传统架构将自动驾驶系统分为感知（检测、跟踪、定位）、预测（行为预测、轨迹预测）、规划（路径规划、决策）和控制（横纵向控制）四个独立模块，每个模块独立开发和优化。端到端架构则将这些功能融合到一个统一的网络中。

**为什么学**：只有深入理解传统方案的每一个模块及其接口设计，才能理解端到端方案在信息传递和优化目标上的优势。很多面试和组会讨论都会涉及这个对比。

**学到什么程度**：能说出传统方案中每个模块的输入输出接口（如感知模块输出3D检测框和跟踪ID，预测模块输出多模态未来轨迹），理解接口设计本身就是信息瓶颈。

**推荐资源**：
- 书籍：《Autonomous Driving: A Multi-robot Perspective》相关章节，或者直接阅读 Apollo 官方文档中的系统架构部分
- 开源项目：Apollo（百度）或 Autoware（Tier IV）的官方代码仓库，快速浏览其模块划分和数据流设计
- 论文：Hu et al., "Planning-oriented Autonomous Driving" (CVPR 2023) —— UniAD的论文，其引言部分对传统方案的问题分析非常经典

**检验标准**：能在白板上完整画出Apollo系统的模块架构图，标注每个模块的输入输出数据格式，并对比端到端方案省去了哪些中间步骤。

### 1.3 为什么端到端是趋势

**学什么**：工业界和学术界转向端到端的原因：（1）数据驱动的方式能发现人工规则难以覆盖的 corner case；（2）避免模块间的信息损失；（3）统一优化目标使得整体性能可能超越独立优化各模块之和；（4）大规模数据和算力的成熟使得端到端训练成为可能。

**为什么学**：理解行业趋势对选择研究方向至关重要。Tesla 2023年宣布将FSD转向端到端方案是一个标志性事件。

**学到什么程度**：能列举至少3个工业界的端到端案例（Tesla FSD v12、Wayve、商汤绝影等），并说出它们各自采用的技术路线。

**推荐资源**：
- 公开演讲：Andrej Karpathy 在 CVPR 2022 Workshop 上的演讲 "Tesla AI"（YouTube/B站均有搬运）—— 讲解了Tesla感知系统的演进思路
- 新闻/分析：搜索"Tesla FSD v12 end to end"相关技术分析文章
- 论文：Hu et al., "Planning-oriented Autonomous Driving" (CVPR 2023 Best Paper) —— 明确提出"以规划为导向"的自动驾驶研究范式

**检验标准**：能够用5分钟向他人讲述为什么Tesla从模块化转向端到端、这个转变背后的技术逻辑是什么。

### 1.4 端到端的几种范式

**学什么**：端到端自动驾驶的三种主要技术范式——
（a）**纯映射式（Direct Mapping）**：传感器输入直接映射到控制输出，代表是早期的行为克隆方法（DAVE-2）。
（b）**中间表征式（Intermediate Representation）**：在网络内部构建可解释的中间表示（如BEV特征、检测框、高精地图向量），最终仍输出规划轨迹，代表是UniAD、VAD。
（c）**世界模型式（World Model）**：学习环境的动力学模型，能够想象"如果我执行某个动作，未来世界会怎样变化"，代表是MILE、GAIA-1、DriveDreamer。

**为什么学**：这三种范式是当前端到端研究的主要分支，理解它们的优缺点有助于选择研究方向。纯映射式简单但缺乏可解释性；中间表征式兼具可解释性和性能，是当前主流；世界模型式最具潜力但最复杂。

**学到什么程度**：能对三种范式进行结构化对比（可解释性、训练难度、数据需求、性能上限），并说出每种范式当前最前沿的工作。

**推荐资源**：
- 综述论文：Li et al., "A Survey on Autonomous Driving Datasets: Statistics, Annotation Quality, and a Future Outlook" (IEEE TITS 2024)
- 综述论文：Tian et al., "A Survey on Vision-based 3D Object Detection for Autonomous Driving" —— 虽然不是直接关于端到端，但帮你理解感知部分的演进
- 中文综述：知乎或微信公众号上搜索"端到端自动驾驶综述2024"，有多篇整理

**检验标准**：能画出三种范式的架构对比图，每种至少举出2篇代表性论文，并说明中间表征式为什么是当前工程落地的主流选择。

---

## Part 2: 代表性方法详解

### 2.1 UniAD —— 统一感知-预测-规划

**学什么**：UniAD（Unified Autonomous Driving）将检测、跟踪、建图、运动预测、占用预测和规划全部集成到一个基于Transformer的端到端框架中。核心思想是"以规划为导向"，所有上游任务都为最终的规划服务。它使用统一的查询（query）机制在各任务间传递信息，避免了传统方案中模块间的信息瓶颈。

**为什么学**：UniAD是CVPR 2023 Best Paper，是端到端自动驾驶领域的里程碑工作。它的设计思路（多任务统一框架、查询式信息传递）深刻影响了后续大量工作。

**学到什么程度**：
- 理解UniAD的六个子任务及其依赖关系（检测→跟踪→建图→运动预测→占用预测→规划）
- 理解查询（query）如何在不同任务间传递信息
- 能够在nuScenes数据集上理解其规划指标（L2误差、碰撞率）
- 不需要从零手写代码，但要能读懂核心前向传播流程

**推荐资源**：
- 论文：Hu et al., "Planning-oriented Autonomous Driving" (CVPR 2023) —— 精读，重点看Section 3的方法部分
- GitHub：https://github.com/OpenDriveLab/UniAD —— 官方代码，基于mmdetection3d框架
- B站/YouTube：搜索"UniAD 论文解读"，有多个中文学术解读视频
- 补充阅读：BEVFormer (Li et al., ECCV 2022) —— UniAD的感知骨干依赖BEVFormer，理解BEVFormer对读懂UniAD至关重要
- GitHub：https://github.com/fundamentalvision/BEVFormer

**检验标准**：能在白板上画出UniAD的完整架构图（六个模块及连接关系），能解释为什么"统一训练"比"分模块训练再拼接"效果更好。

### 2.2 VAD —— 向量化场景表示与端到端规划

**学什么**：VAD（Vectorized Scene Representation for Autonomous Driving）用向量化的方式表示驾驶场景——将地图元素、障碍物轨迹等都编码为向量集合（一组点序列），而不是栅格化的BEV特征图。这种表示更紧凑、更符合驾驶场景的结构化特性。VAD在nuScenes上的规划性能超越了UniAD。

**为什么学**：VAD代表了"向量化表示"这一重要技术趋势，其设计简洁且有效，非常适合工程落地。理解向量化表示对于后续很多工作（SparseDrive、PARA-Drive等）都有帮助。

**学到什么程度**：
- 理解向量化地图表示（向量化车道线、人行横道等）的编码方式
- 理解VAD如何将规划问题转化为向量集合上的优化
- 能在nuScenes数据集上复现基本的规划结果

**推荐资源**：
- 论文：Jiang et al., "VAD: Vectorized Scene Representation for Efficient Autonomous Driving" (ICLR 2024)
- GitHub：https://github.com/hustvl/VAD —— 代码结构清晰，适合学习
- 前置知识：需要了解VectorMapNet (Liu et al., 2022) 或 MapTR (Liao et al., ICLR 2023) 中的地图向量化表示方法
- GitHub：https://github.com/hustvl/MapTR

**检验标准**：能够独立配置VAD的训练环境，在nuScenes数据集上跑通训练和评估流程，并理解其L2和碰撞率指标的含义。

### 2.3 ThinkTwice —— 自适应感知-规划

**学什么**：ThinkTwice提出了一种"在检测器和规划器之间建立高效连接"的方法。核心思想是：规划器不需要完美的感知结果，而需要"对规划有用的"感知信息。它通过一个轻量的适配模块将检测器的输出（包括潜在的误检和漏检）传递给规划器，让规划器自己学会从有噪声的感知结果中做决策。

**为什么学**：现实中的感知系统不可能完美，理解"如何在感知噪声下做鲁棒规划"是工程落地的关键问题。ThinkTwice提供了一个优雅的思路。

**学到什么程度**：理解其两阶段设计（检测+适配+规划），理解为什么直接把检测结果硬编码到规划中会导致脆弱性。能够对比其与UniAD的端到端设计差异。

**推荐资源**：
- 论文：Jia et al., "Think Twice before Driving: Towards Scalable Decoders for End-to-End Autonomous Driving" (CoRL 2023)
- GitHub：https://github.com/jiaxiaosong1011/ThinkTwice
- 补充：了解传统的"感知-规划接口"设计，如检测框列表、占用栅格等常见中间表示

**检验标准**：能解释ThinkTwice中"自适应"的含义，能对比硬接口（deterministic interface）和软接口（learnable interface）的优劣。

### 2.4 GenAD —— 生成式端到端驾驶

**学什么**：GenAD将生成式模型（如VAE、扩散模型）引入端到端驾驶。它不直接回归一条规划轨迹，而是生成多条可能的未来轨迹（捕获多模态性），并通过场景级别的条件生成来提高规划的多样性。

**为什么学**：驾驶场景天然具有多模态性（同一个场景下可能有多种合理的驾驶策略），传统的回归式规划只能给出一条"平均"轨迹，这在关键决策场景下（如变道、让行）是危险的。生成式方法能更好地处理这种不确定性。

**学到什么程度**：理解为什么回归式规划在多模态场景下的局限性，理解生成式方法如何通过采样多条轨迹来覆盖多种可能性。了解扩散模型或条件VAE在轨迹生成中的应用。

**推荐资源**：
- 论文：Zheng et al., "GenAD: Generalized Predictive Model for Autonomous Driving" (CVPR 2024)
- GitHub：https://github.com/OpenDriveLab/UniAD （GenAD与UniAD同属OpenDriveLab系列）
- 前置知识：了解扩散模型基础（Ho et al., "Denoising Diffusion Probabilistic Models", NeurIPS 2020）
- B站：李宏毅老师的扩散模型讲解视频

**检验标准**：能解释"多模态轨迹"的含义，能画出生成式规划与回归式规划在"十字路口直行vs左转"场景下的输出对比示意图。

### 2.5 SparseDrive —— 稀疏表示的端到端

**学什么**：SparseDrive完全抛弃BEV栅格表示，使用纯稀疏的查询（instance query）来统一感知和规划。所有任务（检测、跟踪、建图、运动预测、规划）都通过稀疏查询完成，避免了BEV栅格化带来的计算开销和信息损失。

**为什么学**：稀疏表示是2024年以来的重要趋势，它显著降低了计算成本（不需要构建全局BEV特征图），同时保持甚至提升了性能。对于工程部署来说，稀疏方法更有吸引力。

**学到什么程度**：理解稀疏查询（sparse query）与稠密BEV特征图的区别和各自优劣。理解SparseDrive如何将感知的实例查询直接传递给规划器。

**推荐资源**：
- 论文：Sun et al., "SparseDrive: End-to-end Autonomous Driving via Sparse Representation" (arXiv 2024)
- GitHub：https://github.com/swc-17/SparseDrive
- 前置阅读：SparseFormer系列和PETR系列，理解3D检测中稀疏查询的演进

**检验标准**：能对比SparseDrive与UniAD在特征表示上的差异（稀疏 vs 稠密BEV），并分析各自在计算效率和性能上的trade-off。

### 2.6 PARA-Drive —— 并行化端到端

**学什么**：PARA-Drive提出将传统串行的感知-预测-流水线改为并行执行。核心思想是多个任务可以同时从共享的BEV特征中提取各自所需的信息，不需要严格的时间顺序依赖。

**为什么学**：并行化设计直接提升了推理速度，对实时性要求极高的自动驾驶系统至关重要。同时，并行化也能减少训练中的梯度传播路径，使训练更稳定。

**学到什么程度**：理解串行依赖和并行执行的区别，理解PARA-Drive如何设计任务间的依赖关系（哪些任务必须串行，哪些可以并行）。

**推荐资源**：
- 论文：Liao et al., "PARA-Drive: Parallelized Architecture for Real-time Autonomous Driving" (CVPR 2024)
- GitHub：https://github.com/wzzheng/PARA-Drive
- 补充阅读：Fast-BEV (Yang et al., 2023) —— 了解BEV计算加速的相关工作

**检验标准**：能画出PARA-Drive的并行计算图，标注各任务之间的依赖关系，并与UniAD的串行结构进行延迟（latency）对比分析。

---

## Part 3: 世界模型（World Model）

### 3.1 什么是世界模型在自动驾驶中的应用

**学什么**：世界模型（World Model）是一种学习环境动态的生成模型，它能够根据当前状态和动作预测下一时刻的环境状态。在自动驾驶中，世界模型可以：（1）想象未来可能发生的场景；（2）为规划提供"心理模拟"能力；（3）生成大规模仿真训练数据。

**为什么学**：世界模型是2024-2025年自动驾驶领域最热门的方向之一。Yann LeCun在多次演讲中强调世界模型是通向通用AI的关键，而在自动驾驶中，它有望解决数据稀缺和长尾场景覆盖的核心问题。

**学到什么程度**：理解世界模型的基本框架（状态编码→动作条件→未来状态预测/生成），能区分模型式强化学习中的世界模型与自动驾驶中世界模型的异同。

**推荐资源**：
- 综述：LeCun, "A Path Towards Autonomous Machine Intelligence" (2022 Open Review) —— LeCun的Position Paper，系统阐述了世界模型的理论框架
- 原始概念：Ha & Schmidhuber, "World Models" (NeurIPS 2018) —— 经典世界模型论文，用VAE+RNN学习游戏环境
- 中文讲解：B站搜索"世界模型 自动驾驶"，有多个科普视频

**检验标准**：能画出世界模型的基本框架图（感知编码→状态表征→动力学模型→想象/规划），并解释"在想象空间中做规划"的含义。

### 3.2 GAIA-1、DriveDreamer与MILE

**学什么**：
- **GAIA-1**（Wayve, 2023）：利用视频、文本和动作条件，通过大规模Transformer生成逼真的驾驶视频。它展示了"世界模型"在生成驾驶场景方面的强大能力。
- **DriveDreamer**（2023）：基于扩散模型的世界模型，能够根据交通规则和场景结构生成多样化的驾驶场景视频。
- **MILE**（Wayve, NeurIPS 2022）：Model-Based Imitation Learning，通过学习世界模型在想象空间中进行规划和强化学习。

**为什么学**：这三篇工作分别代表了世界模型在自动驾驶中的三种应用路线：大规模生成式世界模型（GAIA-1）、扩散模型驱动的场景生成（DriveDreamer）、基于世界模型的规划与学习（MILE）。

**学到什么程度**：能区分三种方法的技术路线差异，理解它们各自的优势和局限。重点理解GAIA-1如何利用大规模数据训练生成式世界模型。

**推荐资源**：
- 论文：Hu et al., "GAIA-1: A Generative World Model for Autonomous Driving" (arXiv 2023)
- 论文：Wang et al., "DriveDreamer: Towards Real-world-driven World Models for Autonomous Driving" (ECCV 2024)
- 论文：Wayve, "Model-Based Imitation Learning for Urban Driving" (NeurIPS 2022, MILE)
- GitHub：https://github.com/wzzheng/DriveDreamer
- 博客：Wayve官方博客关于GAIA-1的技术文章

**检验标准**：能对比三种方法在生成质量、条件控制能力、训练数据需求上的差异，并分析哪种路线最接近工程落地。

### 3.3 世界模型用于仿真数据生成

**学什么**：自动驾驶面临严重的长尾分布问题——现实数据中99%的场景是"正常直行"，但系统必须处理那1%的极端情况。世界模型可以生成这些罕见场景的训练数据（如突然冲出的行人、恶劣天气下的驾驶等），这是当前世界模型最实际的应用之一。

**为什么学**：数据是自动驾驶的核心瓶颈。理解世界模型如何用于数据增强和仿真生成，对于工程落地至关重要。

**学到什么程度**：理解世界模型生成仿真数据的完整流程：场景描述→模型生成→数据筛选→下游任务训练。了解生成数据的质量评估方法。

**推荐资源**：
- 论文：Yang et al., "MARS: An Instance-aware, Modular and Realistic Simulator for Autonomous Driving" (CICAI 2023)
- 工具：CARLA仿真器（https://carla.org/）—— 经典的自动驾驶仿真平台，虽非世界模型但理解仿真的基本需求
- 论文：Hu et al., "MagicDrive: Street View Generation with Diverse 3D Geometry Control" (ICLR 2024) —— 用扩散模型生成驾驶场景

**检验标准**：能设计一个利用世界模型生成极端场景数据的实验方案，包括数据生成、质量评估和下游训练的完整流程。

### 3.4 世界模型用于预测和规划

**学什么**：除了生成数据，世界模型还可以直接嵌入规划回路中——在想象空间中"试错"多种驾驶策略，选择最优方案执行。这种方式类似于人类的"心智模拟"（mental simulation）。

**为什么学**：基于世界模型的规划（model-based planning）是端到端自动驾驶中最有潜力的方向之一，它有望结合神经网络的表达能力和经典规划方法的可解释性。

**学到什么程度**：理解"在世界模型中做MPC/蒙特卡洛树搜索"的基本思路，能对比model-based和model-free规划的优劣。

**推荐资源**：
- 论文：Zhang et al., "MCTS-based Interpretable Decision Making for Autonomous Driving" —— 将蒙特卡洛树搜索与神经网络世界模型结合
- 论文：Hafner et al., "Mastering Atari with Discrete World Models" (ICLR 2021, DreamerV2) —— 虽然是游戏领域，但Dreamer系列是世界模型规划的经典范式
- 补充：了解Model Predictive Control (MPC) 的基本原理

**检验标准**：能解释"在想象空间中规划"与"在真实世界中试错"的本质区别，能分析世界模型预测误差对规划质量的影响（model bias问题）。

---

## Part 4: 大模型驱动的自动驾驶

### 4.1 LMDrive —— 语言模型驱动

**学什么**：LMDrive将大语言模型（LLM）引入自动驾驶，用自然语言指令（如"在下一个路口左转"、"注意右边的行人"）作为额外输入，结合传感器数据，让LLM做出驾驶决策。LLM在这里扮演"推理大脑"的角色，利用其世界知识和推理能力处理复杂场景。

**为什么学**：LLM为自动驾驶带来了前所未有的常识推理能力和人机交互方式。LMDrive展示了"语言驱动驾驶"的可行性，是2024年的热门方向。

**学到什么程度**：理解LMDrive的架构设计（视觉编码器→投影层→LLM→驾驶决策），理解语言指令如何与视觉信息融合。

**推荐资源**：
- 论文：Shao et al., "LMDrive: Closed-Loop End-to-End Driving with Large Language Models" (CVPR 2024)
- GitHub：https://github.com/opendrivelab/LMDrive
- 前置知识：了解LLaMA/LLaVA等视觉语言模型的基本架构

**检验标准**：能画出LMDrive的完整架构图，理解视觉token和语言token的融合方式，能分析LLM在驾驶场景中的优势和局限。

### 4.2 DriveGPT

**学什么**：DriveGPT将自动驾驶的决策过程建模为序列预测问题——将历史的驾驶观察和动作编码为token序列，用GPT式的自回归模型预测未来的驾驶动作。这与ChatGPT预测下一个token的范式完全一致。

**为什么学**：这种范式展示了"驾驶即序列预测"的优雅类比，与LLM的技术栈高度重叠。理解这种设计有助于把握自动驾驶与大模型融合的趋势。

**学到什么程度**：理解自回归式驾驶决策的基本框架，能类比GPT的next-token prediction来理解next-action prediction。

**推荐资源**：
- 论文：Mao et al., "A Language Agent for Autonomous Driving" (CoRL 2023, Agent-Driver)
- 论文：Hu et al., "GPT-Driver: Learning to Drive with GPT" (arXiv 2023)
- GitHub：https://github.com/PointsCoder/GPT-Driver

**检验标准**：能将GPT中的"token序列预测"与自动驾驶中的"驾驶动作序列预测"进行结构化类比，并讨论序列预测在安全性要求极高的驾驶场景中的局限性。

### 4.3 多模态大模型在自动驾驶中的应用

**学什么**：以GPT-4V、Gemini为代表的多模态大模型展示了强大的视觉理解能力，研究者开始探索直接用这些模型进行驾驶场景理解和规划。同时，自动驾驶领域也在训练自己的领域专用多模态模型（如DriveVLM）。

**为什么学**：多模态大模型能够理解复杂的交通场景语义（如施工区域、交警手势、非标准交通标志），这是传统感知系统难以覆盖的长尾场景。

**学到什么程度**：了解多模态大模型在自动驾驶中的应用场景（场景理解、异常检测、可解释规划），理解当前的主要局限（推理延迟高、幻觉问题、缺乏驾驶领域知识）。

**推荐资源**：
- 论文：Tian et al., "DriveVLM: The Convergence of Autonomous Driving and Large Vision-Language Models" (arXiv 2024)
- 论文：Wen et al., "On the Road with GPT-4V(ision): Early Explorations of Using Large Multimodal Model for Autonomous Driving" (arXiv 2023)
- 论文：Ding et al., "Hindsight is 20/20: Leveraging Past Traversals to Aid 3D Perception" (ICLR 2024) —— 虽非大模型工作，但展示了知识增强感知的思路
- GitHub：https://github.com/opendrivelab/DriveLM —— DriveLM项目，用语言模型辅助驾驶

**检验标准**：能分析当前多模态大模型在自动驾驶中的三个核心挑战（推理延迟、幻觉、安全性验证），并提出可能的解决方向。

---

## Part 5: 学习建议

### 5.1 端到端方向的就业前景

**学什么**：了解端到端自动驾驶在工业界的落地现状和人才需求。

**为什么学**：选择研究方向需要考虑就业前景。

**学到什么程度**：知道哪些公司在做端到端自动驾驶（Tesla、Wayve、华为、小鹏、蔚来、商汤绝影、地平线等），了解岗位要求（感知算法、规划算法、端到端算法工程师）。

**推荐资源**：
- 招聘网站：在Boss直聘、猎聘上搜索"端到端自动驾驶"相关岗位
- 技术博客：各家公司技术团队的公众号文章（如华为智能驾驶、小鹏汽车自动驾驶等）
- 会议：关注CVPR、ICLR、CoRL、NeurIPS等会议中自动驾驶相关的Workshop

**检验标准**：能列出至少5家在招聘端到端算法工程师的公司，了解其岗位JD中要求的核心技能。

### 5.2 适合作为研究生课题方向吗？

**学什么**：评估端到端自动驾驶作为研究生课题的可行性。

**为什么学**：研究生课题需要兼顾学术创新和工程落地。

**学到什么程度**：明确端到端方向的优势（工业需求大、论文产出快、开源社区活跃）和挑战（算力需求大、实验周期长、需要nuScenes/Waymo等大规模数据集、需要较好的GPU资源）。

**推荐资源**：
- 经验分享：知乎搜索"自动驾驶方向读研"，有大量在读研究生的分享
- 数据集：nuScenes (https://www.nuscenes.org/)、Waymo Open Dataset (https://waymo.com/open/)、CARLA Leaderboard

**检验标准**：能列出端到端方向的3个可行的研究切入点，并评估每个方向的难度、创新空间和工程可行性。

### 5.3 需要的基础知识

**学什么**：进入端到端自动驾驶领域需要具备的知识体系。

**为什么学**：机器人工程背景的学生通常有控制论和ROS基础，但可能在深度学习和计算机视觉方面需要补课。

**学到什么程度**：

**(a) 深度学习基础（必修）**
- CNN、Transformer、注意力机制的原理
- 目标检测（DETR系列）、语义分割的基本方法
- BEV感知的基本概念
- 资源：李宏毅机器学习课程（B站免费）、Stanford CS231n

**(b) 3D视觉与BEV感知（必修）**
- 相机模型、多视角几何、点云处理
- BEV特征构建方法（LSS、BEVFormer等）
- 资源：论文 "LSS: Lift, Splat, Shoot" (ECCV 2020)、"BEVFormer" (ECCV 2022)

**(c) 序列预测与Transformer（重要）**
- 原始Transformer论文 "Attention is All You Need" (NeurIPS 2017)
- DETR (Carion et al., ECCV 2020) —— 查询式检测
- 资源：哈佛NLP的"The Annotated Transformer"教程

**(d) 轨迹预测与规划（重要）**
- 经典规划算法：A*、RRT、MPC
- 数据驱动的轨迹预测：VectorNet、HiVT
- 资源：《Planning Algorithms》(Steven LaValle) 免费在线版

**(e) PyTorch与实验技能（必修）**
- PyTorch基础、分布式训练、混合精度训练
- mmdetection3d / OpenMMLab框架
- 资源：PyTorch官方教程、mmdet3d文档

**检验标准**：能够在nuScenes数据集上独立配置和运行一个BEV感知模型的训练和评估。

### 5.4 推荐的学习路径

**学什么**：系统化的学习时间规划。

**为什么学**：没有计划的学习容易在论文海洋中迷失。

**学到什么程度**：按以下12-16周路径执行——

**第1-2周：入门与全景认知**
- 阅读端到端自动驾驶综述，建立全局认知
- 完成NVIDIA DAVE-2论文精读
- 搭建PyTorch + nuScenes开发环境

**第3-4周：BEV感知基础**
- 精读LSS和BEVFormer论文
- 在mmdet3d上运行BEVFormer代码
- 理解多摄像头BEV特征构建的完整流程

**第5-6周：端到端方法精读（上）**
- 精读UniAD论文，运行官方代码
- 精读VAD论文，理解向量化表示
- 对比两者的设计理念差异

**第7-8周：端到端方法精读（下）**
- 阅读ThinkTwice、SparseDrive、PARA-Drive
- 完成6篇代表作的方法对比表格
- 选定自己最感兴趣的细分方向

**第9-10周：世界模型与大模型**
- 阅读GAIA-1、DriveDreamer、MILE论文
- 阅读LMDrive论文
- 理解世界模型和LLM在驾驶中的应用前景

**第11-12周：动手实践**
- 复现一篇代表性方法的核心结果（建议从VAD开始，代码最清晰）
- 尝试在nuScenes上做一个小的改进实验（如修改规划头结构）

**第13-16周：深入研究**
- 确定研究生课题方向
- 阅读该方向的全部相关论文（20-30篇）
- 撰写文献综述，提炼研究问题

**推荐资源汇总**：
- 论文追踪：Papers With Code 上的 "Autonomous Driving" 标签，GitHub上的 "Awesome Autonomous Driving" 仓库（https://github.com/Hzzone/Awesome-Autonomous-Driving）
- 开源社区：OpenDriveLab（https://github.com/OpenDriveLab）—— 出品了UniAD、GenAD、DriveLM等系列工作，代码质量极高
- 课程：Stanford CS231n（深度学习与计算机视觉）、赵虚左老师的自动驾驶课程（B站）
- 数据集：nuScenes（必用）、Waymo Open Dataset（规模更大）、CARLA（仿真）
- 社区：微信公众号"自动驾驶之心"、"CVer"，定期推送论文解读

**最终检验标准**：能在硕士开题报告中，清晰地陈述端到端自动驾驶的技术现状、自己选定的研究方向、拟解决的关键问题和初步的技术方案。

---

> 本指南涵盖的论文和资源共计约40项。建议以精读6篇核心论文（DAVE-2、BEVFormer、UniAD、VAD、SparseDrive、LMDrive）为主线，其余作为扩展阅读。工程实践方面，以nuScenes数据集和OpenMMLab框架为核心工具链，尽快进入"读代码-跑实验-改实验"的正循环。

---

# 模块八：研零到毕业具体周计划表

# 自动驾驶感知与规划方向 -- 硕士研究生周学习计划

## 个人情况总览

| 项目 | 内容 |
|------|------|
| 本科 | 双非机器人工程 |
| 硕士 | 985船舶与海洋工程（2026-2029） |
| 基础 | 自动控制原理 + 现代控制理论 |
| 导师资源 | 云洲智能（无人船）、字节跳动合作 |
| 目标方向 | 自动驾驶感知与规划 |
| 偏好 | 工程落地 |

---

## 全局节奏建议

### 每日时间分配（工作日）

| 时间段 | 内容 | 时长 |
|--------|------|------|
| 9:00-12:00 | 核心学习（课程/书籍/论文） | 3h |
| 14:00-17:00 | 代码实践（项目/实验） | 3h |
| 18:30-20:00 | LeetCode刷题 | 1.5h |
| 20:30-21:30 | 论文阅读或技术博客 | 1h |

### 周末安排

| 时间段 | 内容 | 时长 |
|--------|------|------|
| 周六上午 | 本周知识整理 + 笔记总结 | 3h |
| 周六下午 | 项目/实验推进 | 3h |
| 周日上午 | 论文精读（2篇） | 3h |
| 周日下午 | LeetCode专题 + 复盘 | 2h |
| 周日晚上 | 下周计划制定 | 1h |

### LeetCode刷题节奏

- 工作日：每天2题（1道中等 + 1道简单/中等）
- 周末：每天1道困难或专题练习
- 每周总量：约8-10题
- 优先专题：数组/链表 -> 二叉树 -> 图 -> 动态规划 -> 滑动窗口 -> BFS/DFS

### 论文阅读节奏

- 研零阶段：每周精读1篇（做笔记、画流程图）
- 研一阶段：每周精读2篇 + 泛读2-3篇
- 研二阶段：每周精读1篇 + 泛读3-5篇（跟前沿）

### 技术博客

- 平台：CSDN 或 个人GitHub Pages
- 频率：每两周发1篇（复现笔记/项目总结/算法解析）
- 内容：代码可运行、图文并茂、有对比实验

---

## 阶段一：研零入学前（现在 ~ 开学，约4个月 / 16周）

### 第1周：C++强化 -- 智能指针与RAII

**学习内容**：unique_ptr/shared_ptr/weak_ptr 用法、RAII模式、自定义删除器

**使用资源**：
- 书籍：《Effective Modern C++》条款18-22
- 视频：侯捷C++新特性（B站）
- 文档：cppreference 智能指针章节

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | RAII原理学习，手写简单RAII类 |
| 周二 | unique_ptr 学习 + 5个练习 |
| 周三 | shared_ptr/weak_ptr 学习 + 循环引用问题 |
| 周四 | 自定义删除器、智能指针与容器配合 |
| 周五 | 复现一个简单对象池（用shared_ptr） |
| 周六 | 整理笔记 + 写博客"智能指针完全指南" |
| 周日 | LeetCode 2题（链表类） + 下周计划 |

**本周产出**：GitHub仓库 `cpp-modern-features`，含智能指针用例代码 + 1篇技术博客

**检验标准**：能独立用智能指针管理一个类的生命周期，无内存泄漏（valgrind验证）

---

### 第2周：C++强化 -- 移动语义、Lambda、多线程

**学习内容**：右值引用、std::move、完美转发、Lambda表达式、std::thread/mutex/condition_variable

**使用资源**：
- 书籍：《Effective Modern C++》条款23-25, 31-34
- 视频：侯捷C++右值引用与移动语义
- 文档：cppreference thread 章节

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | 右值引用与移动语义原理 |
| 周二 | std::move/std::forward 实践 |
| 周三 | Lambda表达式 + std::function/std::bind |
| 周四 | std::thread/mutex 基础 + 生产者消费者模型 |
| 周五 | condition_variable + 线程池手写 |
| 周六 | 整理C++专题笔记 + 博客"移动语义实战" |
| 周日 | LeetCode 2题 + 下周计划 |

**本周产出**：手写简易线程池（C++17），GitHub提交

**检验标准**：线程池可正确并发执行任务，无死锁、无数据竞争（ThreadSanitizer验证）

---

### 第3周：Python进阶 + NumPy

**学习内容**：Python进阶语法（生成器/装饰器/上下文管理器）、NumPy数组操作/广播/向量化

**使用资源**：
- 书籍：《Python Cookbook》第1-7章重点
- 视频：莫烦Python NumPy教程
- 文档：NumPy官方quickstart

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | 生成器、迭代器、__magic methods__ |
| 周二 | 装饰器（带参/类装饰器） |
| 周三 | 上下文管理器 + 异常处理模式 |
| 周四 | NumPy基础：ndarray、索引、切片 |
| 周五 | NumPy进阶：广播机制、向量化运算 |
| 周六 | 实战：用NumPy实现KNN分类（不用sklearn） |
| 周日 | LeetCode 2题（Python实现） + 下周计划 |

**本周产出**：NumPy实战笔记 + 手写KNN分类器

**检验标准**：KNN在iris数据集上准确率>95%，代码无for循环处理向量

---

### 第4周：Python + OpenCV基础

**学习内容**：OpenCV图像读写/变换/滤波/边缘检测/轮廓/相机标定基础

**使用资源**：
- 文档：OpenCV官方Python Tutorials
- 课程：CS231n课程作业（环境搭建）
- GitHub：opencv-python 官方samples

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | OpenCV安装与图像基本操作（读取/显示/保存/色彩空间） |
| 周二 | 图像变换（缩放/旋转/仿射/透视） |
| 周三 | 滤波（均值/高斯/中值/双边）+ 形态学操作 |
| 周四 | 边缘检测（Canny）+ 轮廓检测与绘制 |
| 周五 | 相机标定 + 畸变矫正实践 |
| 周六 | 项目：车道线检测（传统方法） |
| 周日 | 整理笔记 + LeetCode 2题 |

**本周产出**：车道线检测Demo（传统方法，基于颜色阈值+霍夫变换）

**检验标准**：在提供的测试视频上能正确检测车道线并叠加显示

---

### 第5周：深度学习基础 -- 神经网络原理与PyTorch入门

**学习内容**：前向传播、反向传播、损失函数、优化器、PyTorch基本操作

**使用资源**：
- 视频：李宏毅机器学习（前5讲）
- 书籍：《动手学深度学习》（d2l）第3-6章
- 官方：PyTorch官方60分钟入门教程

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | 神经网络数学基础（链式求导、梯度下降） |
| 周二 | PyTorch张量操作 + autograd机制 |
| 周三 | 用PyTorch手写两层MLP（MNIST分类） |
| 周四 | 学习率调度、Batch Normalization、Dropout |
| 周五 | 用TensorBoard可视化训练过程 |
| 周六 | 对比实验：不同优化器/学习率的收敛对比 |
| 周日 | 整理笔记"神经网络入门到PyTorch实战" + LeetCode 2题 |

**本周产出**：MNIST分类器（准确率>98%）+ 训练可视化截图 + 博客

**检验标准**：能从零手写MLP训练流程，不看教程独立完成forward/backward/step

---

### 第6周：深度学习进阶 -- CNN与经典网络

**学习内容**：卷积操作、池化、经典网络（AlexNet/VGG/ResNet）、迁移学习

**使用资源**：
- 视频：CS231n 第5-9讲
- 书籍：《动手学深度学习》第7-8章
- 论文：ResNet原始论文（He et al., 2015）

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | 卷积层原理 + 手算感受野 |
| 周二 | 实现简单CNN（LeNet风格）训练CIFAR-10 |
| 周三 | 阅读ResNet论文 + 残差连接原理 |
| 周四 | 用PyTorch实现ResNet18并训练 |
| 周五 | 迁移学习：用预训练ResNet做花卉分类 |
| 周六 | 数据增强实验对比 + 整理笔记 |
| 周日 | LeetCode 2题 + 周复盘 |

**本周产出**：ResNet18花卉分类（Kaggle数据集，准确率>90%）+ ResNet论文笔记

**检验标准**：能说清残差连接的数学原理，并独立实现迁移学习pipeline

---

### 第7周：目标检测入门 -- YOLO系列原理

**学习内容**：目标检测基础概念（IoU/NMS/mAP）、YOLO系列演进、YOLOv8架构

**使用资源**：
- 论文：YOLOv1 原始论文 + YOLOv8 官方文档
- GitHub：ultralytics/ultralytics
- 博客：目标检测综述（B站/知乎高质量文章）

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | 目标检测基本概念（Anchor/NMS/IoU/mAP） |
| 周二 | 阅读YOLOv1论文，理解核心思想 |
| 周三 | YOLOv8架构学习（Ultralytics文档） |
| 周四 | 环境搭建 + YOLOv8预训练模型推理测试 |
| 周五 | 数据标注工具学习（LabelImg/Roboflow） |
| 周六 | 自定义数据集制作（收集+标注100+图片） |
| 周日 | LeetCode 2题 + 整理YOLO系列对比笔记 |

**本周产出**：自定义数据集（5类以上）+ YOLO系列技术笔记

**检验标准**：能清晰画出YOLOv8网络结构图，理解每个模块的作用

---

### 第8周：YOLOv8训练与优化

**学习内容**：YOLOv8训练自定义数据集、数据增强、模型评估、ONNX导出

**使用资源**：
- GitHub：ultralytics/ultralytics 官方教程
- 文档：Ultralytics 使用文档
- 工具：Roboflow数据增强

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | YOLOv8训练配置（超参数/数据格式/目录结构） |
| 周二 | 开始训练，监控loss曲线 |
| 周三 | 评估模型（mAP/PR曲线/FPS） |
| 周四 | 数据增强实验对比（Mosaic/MixUp/Hsv） |
| 周五 | 模型导出（ONNX）+ TensorRT部署推理 |
| 周六 | 撰写完整项目README + 整理实验报告 |
| 周日 | LeetCode 2题 + 周复盘 |

**本周产出**：YOLOv8自定义数据集训练全流程 + ONNX推理Demo + 项目GitHub仓库

**检验标准**：mAP@0.5 > 70%（自定义数据集），ONNX推理速度满足实时性要求

---

### 第9周：ROS2入门

**学习内容**：ROS2架构、节点/话题/服务/动作、Colcon构建、Launch文件

**使用资源**：
- 官方：ROS2 Humble 官方Tutorial
- 书籍：《ROS2入门到实践》
- GitHub：ros2/examples

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | ROS2安装（Ubuntu 22.04 + Humble）+ 架构总览 |
| 周二 | 编写第一个ROS2 Python节点 + C++节点 |
| 周三 | 话题通信（Publisher/Subscriber）实践 |
| 周四 | 服务通信（Service/Client）+ 动作通信（Action） |
| 周五 | Launch文件编写 + 参数配置 |
| 周六 | 项目：多节点协同（传感器模拟+处理+显示） |
| 周日 | 整理笔记 + LeetCode 2题 |

**本周产出**：ROS2基础节点集合（GitHub仓库）+ 架构笔记

**检验标准**：能独立创建ROS2功能包，编写发布/订阅节点并正常通信

---

### 第10周：ROS2 + Gazebo仿真

**学习内容**：Gazebo基础、URDF/SDF模型、ROS2-Gazebo桥接、仿真导航

**使用资源**：
- 官方：Gazebo Sim Tutorial + Nav2文档
- GitHub：ros-navigation/navigation2
- 视频：古月居ROS2系列教程

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | Gazebo安装 + 基本场景搭建 |
| 周二 | URDF/SDF模型编写（机器人建模） |
| 周三 | ROS2-Gazebo联合仿真配置 |
| 周四 | 传感器插件（激光雷达/IMU/相机） |
| 周五 | Nav2导航框架入门（建图+导航） |
| 周六 | 项目：机器人在Gazebo中自主避障导航 |
| 周日 | 整理笔记 + LeetCode 2题 |

**本周产出**：Gazebo仿真导航Demo + 视频录屏 + 笔记

**检验标准**：机器人能在Gazebo中从A点自主导航到B点，自动避障

---

### 第11周：SLAM入门 -- 视觉SLAM基础上

**学习内容**：视觉里程计（特征点法/直接法）、相机模型、对极几何

**使用资源**：
- 书籍：《视觉SLAM十四讲》（高翔）第1-7讲
- GitHub：gaoxiang12/slambook2
- 视频：高翔SLAM公开课（B站）

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | SLAM问题概述 + 三维空间刚体运动（李群/李代数） |
| 周二 | 相机模型与图像形成（第5章） |
| 周三 | 特征点提取与匹配（ORB/SIFT实践） |
| 周四 | 对极几何与PnP求解（第7章） |
| 周五 | 代码实践：slambook2 ch7 特征匹配实验 |
| 周六 | 手写简单视觉里程计（特征点法） |
| 周日 | 整理笔记（画图）+ LeetCode 2题 |

**本周产出**：视觉里程计代码 + SLAM数学基础笔记

**检验标准**：能在TUM数据集上用特征点法估计相机轨迹（结果可视化）

---

### 第12周：SLAM入门 -- ORB-SLAM3跑通与后端优化

**学习内容**：后端优化（BA/g2o）、回环检测、ORB-SLAM3编译运行

**使用资源**：
- 书籍：《视觉SLAM十四讲》第8-12讲
- GitHub：UZ-SLAMLab/ORB_SLAM3
- 工具：g2o/GTSAM

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | 后端优化原理（Bundle Adjustment） |
| 周二 | g2o库入门 + slambook2 ch10代码 |
| 周三 | 回环检测原理（词袋模型/DBoW2） |
| 周四 | ORB-SLAM3编译安装（踩坑记录） |
| 周五 | ORB-SLAM3在TUM/KITTI数据集上运行 |
| 周六 | 分析ORB-SLAM3代码架构 + 对比纯视觉里程计 |
| 周日 | 整理SLAM全流程笔记 + LeetCode 2题 |

**本周产出**：ORB-SLAM3运行报告（含轨迹精度对比）+ 博客"ORB-SLAM3编译与运行全记录"

**检验标准**：ORB-SLAM3在TUM RGB-D数据集上ATE < 3cm

---

### 第13-14周：综合项目 -- ROS2自主导航系统（上）

**学习内容**：整合感知+规划+控制，构建ROS2自主导航系统

**使用资源**：
- GitHub：navigation2（Nav2框架）
- 框架：行为树（BehaviorTree.CPP）
- 参考：autoware.universe（自动驾驶全栈参考）

**每日安排**：

| 日期 | 任务 |
|------|------|
| 周一 | 系统设计：传感器配置/模块划分/话题定义 |
| 周二 | 搭建Gazebo仿真环境（含动态障碍物） |
| 周三 | 感知模块：LiDAR点云处理（PCL基础） |
| 周四 | 感知模块：障碍物聚类与检测 |
| 周五 | 规划模块：A* / Dijkstra全局路径规划 |
| 周六 | 规划模块：DWA局部路径规划 |
| 周日 | 整理周进度 + LeetCode 2题 |

**第14周**：

| 日期 | 任务 |
|------|------|
| 周一 | 控制模块：PID横向+纵向控制 |
| 周二 | 模块集成与调试 |
| 周三 | 添加YOLOv8视觉检测模块（从第8周复用） |
| 周四 | 多传感器融合（LiDAR + Camera简单融合） |
| 周五 | 系统联调与bug修复 |
| 周六 | 性能测试（成功率/响应时间） |
| 周日 | 整理笔记 + LeetCode 2题 |

**两周产出**：完整ROS2自主导航系统（Gazebo仿真）+ 架构文档 + Demo视频

**检验标准**：机器人在含动态障碍物的环境中成功完成导航任务，成功率>80%

---

### 第15-16周：综合项目 -- ROS2自主导航系统（下）+ 研零总结

**学习内容**：项目优化、文档完善、研零阶段总结

**每日安排**：

| 日期 | 任务 |
|------|------|
| 第15周周一 | 添加行为树决策逻辑（多任务切换） |
| 第15周周二 | 地图服务（自动生成/加载地图） |
| 第15周周三 | 性能优化（CPU/内存分析 + 优化） |
| 第15周周四 | 写完整项目文档（README/架构图/使用说明） |
| 第15周周五 | 录制Demo视频 + 剪辑 |
| 第15周周六 | 整理研零阶段全部笔记/代码仓库 |
| 第15周周日 | 撰写研零学习总结博客 |
| 第16周周一 | 复盘所有项目，整理可复用代码模块 |
| 第16周周二 | 制作个人Portfolio页面（GitHub Pages） |
| 第16周周三 | 整理论文阅读笔记清单 |
| 第16周周四 | 制定研一上学期详细计划 |
| 第16周周五 | 查阅导师研究方向相关论文（5篇以上） |
| 第16周周六 | 休息/运动 |
| 第16周周日 | 迎接开学 |

**两周产出**：ROS2导航项目完整版 + GitHub Pages个人主页 + 研一学习计划

**检验标准**：个人GitHub至少6个有质量的仓库，个人主页可访问，所有项目README完整

---

## 阶段二：研一上（开学 ~ 寒假，约4-5个月）

### 每周固定安排

| 内容 | 频率 | 时长 |
|------|------|------|
| 导师课题 | 每天上午为主 | 3-4h/天 |
| 云洲智能技术调研 | 每周2-3次 | 2h/次 |
| SLAM/LiDAR深入 | 每周3次 | 2h/次 |
| LeetCode | 每天 | 1.5h |
| 论文阅读 | 每周精读2篇 | 4h |

### 第17-20周：LiDAR感知深入

**学习内容**：
- PCL点云处理（滤波/分割/聚类/特征提取）
- 点云配准（ICP/NDT算法）
- LiDAR里程计（LOAM/LeGO-LOAM）

**使用资源**：
- 书籍：《点云库PCL从入门到精通》
- 论文：LOAM, LeGO-LOAM, LIO-SAM
- GitHub：RobustFieldAutonomyLab/LeGO-LOAM, TixiaoShan/LIO-SAM

**产出**：
- LOAM/LeGO-LOAM在公开数据集上的运行报告
- 点云处理工具库（C++/ROS2）
- 博客："LiDAR SLAM从原理到实践"

**检验标准**：能独立运行并修改LeGO-LOAM，理解特征提取与匹配全流程

### 第21-24周：路径规划算法

**学习内容**：
- 图搜索：A*、Dijkstra、JPS
- 采样方法：RRT/RRT*
- 优化方法：Lattice Planner
- 决策规划：有限状态机/决策树

**使用资源**：
- 书籍：《Planning Algorithms》（LaValle）重点章节
- 课程：Coursera "Motion Planning for Self-Driving Cars"（多伦多大学）
- GitHub：geho/zl_planner, Apollo规划模块

**产出**：
- 路径规划算法合集（A*/RRT*/Lattice Planner对比实验）
- 课程作业完整提交
- 博客："自动驾驶路径规划算法全解析"

**检验标准**：能实现至少3种规划算法，在同一场景下对比成功率/路径长度/计算时间

### 第25-28周：深入导师课题 + 云洲智能

**学习内容**：
- 无人船自主航行技术栈调研
- 水上环境感知特殊性（水面反射/波浪/光照变化）
- 结合课题方向选定研究切入点

**产出**：
- 文献综述初稿（3000字以上）
- 课题技术方案文档
- 云洲智能相关技术调研报告

**检验标准**：能在组会上清晰汇报文献综述，获得导师对研究方向的认可

### 持续任务

- LeetCode：累计完成150题（重点：图论/动态规划/二叉树）
- 论文阅读：累计30篇以上（SLAM+感知+规划方向）
- 技术博客：累计8篇以上

---

## 阶段三：研一下（寒假 ~ 暑假，约5-6个月）

### 每周固定安排

| 内容 | 频率 | 时长 |
|------|------|------|
| 导师课题研究 | 工作日为主 | 4-5h/天 |
| 3D检测/BEV感知学习 | 每周3次 | 2h/次 |
| 论文写作 | 每周2-3次 | 2h/次 |
| LeetCode | 每天 | 1h |
| 论文阅读 | 每周精读2篇+泛读3篇 | 5h |

### 第29-32周：3D目标检测

**学习内容**：
- 3D检测基础：点云表示、3D BBox、评价指标
- 单阶段方法：PointPillars
- 两阶段方法：PV-RCNN
- 多模态融合：PointPainting/TransFusion

**使用资源**：
- 论文：PointPillars, PV-RCNN, TransFusion
- 框架：OpenPCDet
- 数据集：KITTI, nuScenes

**产出**：
- PointPillars在KITTI上的复现结果
- 3D检测综述笔记
- 博客："3D目标检测入门到PointPillars复现"

**检验标准**：KITTI Car Moderate AP > 70%

### 第33-36周：BEV感知

**学习内容**：
- BEV表示原理（LSS / BEVFormer）
- BEV空间下的检测与分割
- 时序BEV（BEVDet / BEVFormer）

**使用资源**：
- 论文：LSS, BEVFormer, BEVDet, UniAD
- GitHub：open-mmlab/mmdetection3d

**产出**：
- BEVFormer代码阅读笔记
- BEV感知技术对比分析文档

**检验标准**：能清晰解释BEV空间转换的数学原理，跑通BEVFormer推理demo

### 第37-42周：第一篇论文准备

**学习内容**：
- 论文写作方法（结构/逻辑/图表/公式）
- 实验设计与消融实验
- 英文论文写作基础

**使用资源**：
- 书籍：《学术论文写作指南》
- 参考：目标期刊/会议的高引论文结构
- 工具：Overleaf, draw.io, matplotlib

**产出**：
- 论文初稿（或至少方法+实验部分）
- 实验结果表格与可视化图表
- 导师审阅修改

**检验标准**：论文初稿完成并提交导师审阅，实验结果具有说服力

### 持续任务

- LeetCode：累计完成250题
- 论文阅读：累计60篇以上
- 技术博客：累计12篇以上

---

## 阶段四：研一暑假（暑期实习，约2-3个月）

### 实习目标

- **首选**：字节跳动（自动驾驶/智能驾驶团队）
- **备选**：小马智行/文远知行/蔚来/理想/华为车BU
- **利用导师资源**：字节合作项目优先争取内推

### 实习期间学习策略

| 周次 | 策略 |
|------|------|
| 第1-2周 | 快速理解业务代码库、技术栈、开发流程 |
| 第3-6周 | 高质量完成分配任务，主动学习工业级代码规范 |
| 第7-8周 | 总结实习项目技术方案，准备转正答辩材料 |

### 每日安排（实习期）

| 时间 | 内容 |
|------|------|
| 工作日 | 全力投入实习工作，下班后整理当日技术笔记 |
| 晚上 | LeetCode 1题（保持手感） |
| 周末 | 复盘本周实习收获 + 论文推进 + 准备下周工作 |

### 重点积累

- 工业级代码规范（代码review学到的）
- 工程问题解决方法论（调试/性能优化/系统设计）
- 实习项目的技术文档（秋招面试素材）
- 行业认知：自动驾驶技术栈全景图

### 产出

- 实习项目技术文档
- 实习总结报告
- 秋招简历初稿

---

## 阶段五：研二上（开学 ~ 寒假，约4-5个月）

### 每周固定安排

| 内容 | 频率 | 时长 |
|------|------|------|
| 论文投稿/修改 | 每天 | 3-4h |
| BEV/端到端深入 | 每周3次 | 2h/次 |
| LeetCode | 每天 | 1h |
| 第二次实习准备 | 每周2次 | 2h/次 |

### 第43-46周：论文投稿

**学习内容**：
- 论文修改与润色
- 投稿流程（选刊/格式/提交/回复审稿意见）
- 学术海报/报告准备

**产出**：
- 论文投稿完成
- 审稿意见回复

**检验标准**：论文成功提交至目标会议/期刊

### 第47-52周：端到端自动驾驶

**学习内容**：
- 端到端架构：UniAD, VAD, PARA-Drive
- 大模型驱动的规划：DriveGPT4, LMDrive
- World Model用于自动驾驶

**使用资源**：
- 论文：UniAD, VAD, End-to-End Autonomous Driving (综述)
- GitHub：OpenDriveLab/UniAD

**产出**：
- 端到端自动驾驶技术综述笔记
- UniAD代码阅读报告
- 博客："端到端自动驾驶：从感知到规划的统一架构"

**检验标准**：能清晰讲解UniAD的pipeline，理解query-based方法在自动驾驶中的应用

### 第53-56周：第二次实习准备

**学习内容**：
- 秋招技术面试准备（八股文+项目深挖）
- 简历优化（针对不同岗位定制）
- 模拟面试

**产出**：
- 精修简历3个版本（感知/规划/全栈）
- 面试八股文整理（自动驾驶方向）
- 项目梳理文档（STAR法则）

---

## 阶段六：研二下 ~ 研三（实习 + 秋招 + 论文）

### 第57-64周：第二次实习

**目标**：争取头部自动驾驶公司（小马智行/华为/蔚来等）

**策略**：
- 偏感知岗：强化3D检测/BEV/Transformer相关经验
- 偏规划岗：强化运动规划/决策/仿真相关经验
- 实习期间同步准备秋招

### 第65-72周：秋招全力冲刺

**每周安排**：

| 内容 | 频率 | 时长 |
|------|------|------|
| 算法题 | 每天 | 1.5h |
| 八股文复习 | 每天 | 1h |
| 项目深挖准备 | 每周3次 | 1.5h |
| 模拟面试 | 每周1次 | 1.5h |
| 论文完善 | 每周3次 | 2h |

**秋招投递节奏**：

| 时间 | 动作 |
|------|------|
| 7-8月 | 提前批投递（字节/华为/蔚来等） |
| 8-9月 | 正式批投递（扩大范围） |
| 9-10月 | 面试高峰期（保持状态） |
| 10-11月 | offer比较与决策 |

### 第73-80周：论文完善 + 毕业准备

- 论文根据审稿意见修改并重新投稿
- 毕业论文撰写
- 答辩准备

---

## 附录：核心资源清单

### 书籍

| 书名 | 用途 |
|------|------|
| 《Effective Modern C++》 | C++进阶 |
| 《动手学深度学习》 | 深度学习入门 |
| 《视觉SLAM十四讲》 | SLAM入门 |
| 《Planning Algorithms》 | 规划算法 |

### GitHub仓库

| 仓库 | 用途 |
|------|------|
| ultralytics/ultralytics | YOLOv8 |
| UZ-SLAMLab/ORB_SLAM3 | 视觉SLAM |
| TixiaoShan/LIO-SAM | LiDAR SLAM |
| open-mmlab/mmdetection3d | 3D检测 |
| OpenDriveLab/UniAD | 端到端 |
| ros-navigation/navigation2 | ROS2导航 |

### 课程

| 课程 | 来源 |
|------|------|
| 李宏毅机器学习 | 台大/B站 |
| CS231n | Stanford |
| Motion Planning for Self-Driving Cars | 多伦多大学/Coursera |

### LeetCode刷题规划

| 阶段 | 重点专题 | 目标题数 |
|------|----------|----------|
| 研零 | 数组/链表/二叉树/DFS/BFS | 80题 |
| 研一上 | 图论/动态规划/贪心 | 150题 |
| 研一下 | 滑动窗口/双指针/回溯 | 250题 |
| 研二上 | 高级DP/设计题/hard题 | 300题 |
| 秋招冲刺 | 综合模拟/面试真题 | 350题 |


---

# 模块九：自动驾驶运动预测（Motion Prediction）

## 自动驾驶运动预测（Motion Prediction）详细学习指南

---

## Part 1: 运动预测概述

### 什么是运动预测

运动预测（Motion Prediction / Trajectory Forecasting）是指：给定自动驾驶车辆周围其他交通参与者（车辆、行人、骑行者等）的历史运动状态，预测它们在未来一段时间（通常 3~8 秒）内的运动轨迹。

**学什么：** 理解预测任务的正式定义——输入是过去 T_obs 秒的历史轨迹（通常 2 秒），输出是未来 T_pred 秒的预测轨迹（通常 3~6 秒）。每条轨迹由一系列 (x, y) 坐标点组成，通常以 10Hz 采样。

**为什么学：** 这是预测方向的起点。只有清晰理解任务定义，才能判断一个方法是否真的在解决问题、还是在做无意义的改进。

### 在自动驾驶 Pipeline 中的位置

自动驾驶系统通常分为四大模块：感知（Perception） → 预测（Prediction） → 规划（Planning） → 控制（Control）。

- **感知**：负责检测和跟踪周围物体，输出 3D 检测框 + 跟踪 ID。
- **预测**：接收感知输出和高精地图，预测每个交通参与者未来的运动轨迹。
- **规划**：结合预测结果，规划自车的安全且舒适轨迹。
- **控制**：执行规划轨迹。

**学到什么程度：** 理解上下游的数据流向即可。不需要深入感知或控制的细节，但要知道预测模块的输入输出格式。

### 预测的输入与输出

**输入（三类信息）：**
1. **智能体历史轨迹**：每个交通参与者过去 2 秒的位置、速度、朝向、类别（车/人/骑行者）。
2. **高精地图（HD Map）**：车道线中心线、车道边界、人行横道、停车线、交通信号灯状态等，通常用矢量化的折线或多边形表示。
3. **交通信号状态**：红绿灯、停车标志等（部分数据集提供）。

**输出：**
- 未来 3~8 秒的轨迹预测。由于未来存在不确定性，输出通常是 **多模态（multi-modal）** 的——对每个智能体给出 K 条（通常 K=6）候选轨迹及其概率，例如：
  - 轨迹 1：直行，概率 60%
  - 轨迹 2：左转，概率 25%
  - 轨迹 3：右转，概率 15%

**学到什么程度：** 务必查看 Argoverse 2 或 Waymo Open Motion 数据集的具体数据格式（JSON/Protobuf），亲手加载和可视化几条数据，建立直觉。

**推荐资源：**
- Argoverse 2 官方 API：`https://argoverse.github.io/user-guide/getting_started.html`（使用 `argoverse` Python 包加载数据）
- Waymo Open Dataset：`https://waymo.com/open/`

### 为什么预测很难

1. **多模态性（Multi-modality）**：同一个场景中，一个车辆可能直行也可能转弯，无法用单一轨迹表示。
2. **交互性（Interaction）**：车辆之间会博弈——前车减速，后车可能变道。忽略交互会导致预测不合理。
3. **意图不确定性（Intent Uncertainty）**：你无法直接观测到驾驶员的意图，只能从行为推断。
4. **场景复杂性**：复杂的路口、施工区域、没有车道线的区域等都增加难度。

**学到什么程度：** 理解这些挑战的具体含义，并能在看论文时判断某篇工作是在解决哪个挑战。例如 VectorNet 解决场景编码问题，HiVT 解决交互问题。

---

## Part 2: 基于学习的预测方法

以下六篇论文是预测方向的核心脉络，建议按顺序精读。

### 1. VectorNet（CVPR 2020）

**论文：** *VectorNet: Encoding HD Maps and Agent Dynamics from Vectorized Representation*

**核心思想：** 将高精地图和智能体轨迹统一表示为向量（polyline）。每条车道线是一组向量，每段轨迹也是一组向量。所有向量送入一个图神经网络（GNN）进行编码，再用 Transformer 建模全局交互。

**学什么：**
- 向量化场景表示（Polyline Subgraph）：把点级别的特征聚合成段级别的特征。
- 为什么比光栅化（rasterization）方法好：更高效、保留几何信息、无需渲染图像。

**为什么学：** VectorNet 定义了后续几乎所有预测方法的输入表示范式。从此之后，"vectorized representation"成为主流。

**学到什么程度：** 精读论文，理解 polyline 编码过程。复现最好，但至少要能跑通官方代码并可视化。

**资源：**
- 论文：`https://arxiv.org/abs/2005.04259`
- 代码：`https://github.com/uber-research/VectorNet`

### 2. TNT（Target-driven Trajectory Prediction）

**论文：** *TNT: Target-driven Trajectory Prediction*

**核心思想：** 三阶段预测——(1) 在空间中采样一组候选目标点（goal candidates）；(2) 对每个目标点回归一条完整轨迹；(3) 对所有轨迹打分排序。

**学什么：**
- 目标驱动（goal-conditioned）的预测范式：先猜意图（目标点），再生成轨迹。
- 这种"先采样后精炼"的思路对后续工作影响深远。

**为什么学：** 理解"条件预测"的核心思想。后续的 DenseTNT、MTR 等都延续了这个思路。

**学到什么程度：** 理解三阶段流程，能够解释为什么目标驱动比直接回归轨迹更好。

**资源：**
- 论文：`https://arxiv.org/abs/2008.08294`
- 代码：参考 Waymo 的实现

### 3. HiVT（Hierarchical Vector Transformer，ECCV 2022）

**论文：** *HiVT: Hierarchical Vector Transformer for Multi-Agent Motion Prediction*

**核心思想：** 分层建模——先在局部坐标系中编码每个智能体和其附近的车道线，再在全局坐标系中用 Transformer 建模所有智能体之间的交互。

**学什么：**
- **局部-全局分层**：局部特征提取（每个 agent 附近的场景）+ 全局交互（所有 agent 之间的关系）。
- **平移旋转不变性**：在局部坐标系中编码，避免模型对绝对位置的依赖。

**为什么学：** HiVT 在多智能体联合预测上有很好的表现，且代码质量高，适合入门学习。

**学到什么程度：** 精读论文，重点理解坐标系变换和分层交互机制。强烈建议跑代码并分析中间结果。

**资源：**
- 论文：`https://arxiv.org/abs/2205.01170`
- 代码：`https://github.com/ZikangZhou/HiVT`

### 4. MTR（Motion Transformer，NeurIPS 2022）

**论文：** *MTR: Motion Transformer with Global Intention Localization and Local Movement Refinement*

**核心思想：** 两阶段预测——(1) 用一组可学习的运动查询（motion queries）对全局意图进行建模（对应不同的运动模式如直行、左转、右转）；(2) 对每种意图局部细化轨迹。在 Waymo Open Motion 榜单上取得了冠军成绩。

**学什么：**
- **Motion Query**：用 Transformer 的 query 机制表示不同的运动模式。
- **意图与轨迹分离**：全局意图定位（是什么模式）+ 局部运动精炼（在这个模式下具体怎么走）。

**为什么学：** MTR 是目前最有影响力的方法之一，其设计思路（多查询 Transformer）被大量后续工作借鉴。理解 MTR 就理解了当前主流范式。

**学到什么程度：** 精读论文，理解 motion query 的设计。建议在 Waymo 数据集上跑通训练和评估。

**资源：**
- 论文：`https://arxiv.org/abs/2209.13508`
- 代码：`https://github.com/sshaoshuai/MTR`

### 5. QCNet（Query-Centric Trajectory Prediction，ICCV 2023）

**论文：** *QCNet: Query-Centric Trajectory Prediction*

**核心思想：** 进一步改进 MTR 的查询机制，以查询为中心设计场景编码。引入了锚点查询（anchor queries）从数据中学习空间先验，并使用相对坐标系编码。

**学什么：**
- 查询中心范式（Query-Centric）：所有场景信息都围绕查询来组织。
- 锚点机制：用数据驱动的方式代替手工设计的目标点采样。
- 相对位置编码和时间编码的细节。

**为什么学：** 代表了运动预测领域的最新前沿。思路简洁且性能强，适合做为后续研究的 baseline。

**学到什么程度：** 精读论文并跑代码，在 Argoverse 2 上对比不同方法的性能。

**资源：**
- 论文：`https://arxiv.org/abs/2306.10508`
- 代码：`https://github.com/ZikangZhou/QCNet`

### 6. UniTraj（Unified Trajectory Prediction Framework）

**论文：** *UniTraj: A Unified Framework for Scalable Vehicle Trajectory Prediction*

**核心思想：** 提出一个统一的预测框架，可以兼容多种不同的数据集格式、场景编码方式和预测头，方便公平对比不同方法。

**学什么：**
- 统一评估框架的设计理念。
- 不同数据集之间的差异和统一方法。

**为什么学：** 作为研究工具非常有价值，可以帮你快速在多个数据集上对比方法，避免重复造轮子。

**学到什么程度：** 了解框架的使用方法，在自己的研究中作为实验工具。

**资源：**
- 论文：`https://arxiv.org/abs/2403.15098`
- 代码：`https://github.com/vita-epfl/UniTraj`

---

## Part 3: 预测任务的核心要素

### 场景编码（Scene Encoding）

**学什么：**
- **车道线编码**：将 HD Map 中的车道中心线、边界线等用 PointNet 或 GNN 编码为向量特征。关键问题是如何处理不同长度、不同密度的车道线。
- **智能体历史轨迹编码**：用 1D-CNN、LSTM 或 Transformer 对过去 2 秒的轨迹点序列编码。需要处理不同类型的交通参与者（车辆 vs 行人）。
- **交通信号编码**：将红绿灯状态与对应的车道线关联起来，作为条件输入。
- **场景融合**：如何将地图特征和智能体特征融合（拼接、注意力机制、交叉注意力等）。

**为什么学：** 场景编码决定了模型"看到"什么信息。编码质量直接决定预测上限。

**学到什么程度：** 能独立实现一个完整的场景编码器（车道线 + 历史轨迹 → 场景特征图）。重点掌握 VectorNet 和 MTR 的编码方式。

**资源：**
- PointNet：`https://arxiv.org/abs/1612.00593`（基础的点云编码方法）
- LaneGCN：`https://arxiv.org/abs/2007.02205`（车道线图建模）

### 意图预测（Intent Prediction）

**学什么：**
- 意图预测就是判断智能体的高层运动目标：直行/左转/右转/停车/换道。
- 常用方法：(1) 分类头直接预测意图类别；(2) 通过目标点（goal）间接表示意图；(3) 用隐式意图（motion query）表示。
- 意图不确定性建模：同一场景下可能有多种合理的意图，需要概率分布。

**为什么学：** 意图是连接高层语义和低层轨迹的桥梁。好的意图预测能显著提升轨迹质量。

**学到什么程度：** 理解分类式 vs 目标式 vs 隐式三种意图表示方式的优劣。

### 轨迹生成（Trajectory Generation）

**学什么：**
- **回归式生成**：直接用 MLP 回归未来轨迹点（简单但缺乏多模态能力）。
- **锚点/候选式**：先采样目标点，再用回归生成轨迹到该目标（TNT 范式）。
- **多元高斯分布**：用参数化的高斯分布（均值 + 方差）表示每个轨迹点的不确定性。
- **扩散模型（Diffusion Model）**：2023 年后的新趋势，用去噪扩散过程生成轨迹，天然支持多模态。

**推荐资源：**
- MotionDiffuser：`https://arxiv.org/abs/2306.14840`（扩散模型用于运动预测）
- MTR++：`https://arxiv.org/abs/2306.17770`（MTR 的增强版，增加概率回归）

**学到什么程度：** 掌握回归式和锚点式生成，了解扩散模型的基本思想。如果研究方向偏生成模型，需要深入学习扩散模型。

### 交互预测（Multi-Agent Interaction）

**学什么：**
- **联合预测（Joint Prediction）**：同时预测多个智能体的轨迹，考虑它们之间的相互影响。
- **图神经网络（GNN）**：将智能体建模为图节点，边表示交互关系。
- **Transformer 的自注意力**：用注意力机制自动学习交互权重。
- **博弈论视角**：将交互建模为博弈，用纳什均衡等概念解释。

**为什么学：** 在路口、并道等场景中，忽略交互会导致不一致的预测（例如两辆车的预测轨迹发生碰撞）。

**学到什么程度：** 理解交互建模的必要性，能够实现基于 Transformer 或 GNN 的交互模块。

**资源：**
- Scene Transformer（Waymo，2022）：`https://arxiv.org/abs/2206.08756`
- M2I（From Marginal to Joint Predictions）：`https://arxiv.org/abs/2206.05646`

### 条件预测（Conditional Prediction）

**学什么：**
- 以意图/目标为条件生成轨迹：给定"左转"意图，生成左转轨迹。
- 以自车行为为条件：给定自车的规划轨迹，预测其他车辆的反应。
- 条件变分自编码器（CVAE）：用潜变量 z 表示条件，采样不同的 z 得到不同的轨迹。

**为什么学：** 条件预测是连接预测和规划的关键。在闭环仿真中，你需要根据自车行为条件化地预测其他车辆的反应。

**学到什么程度：** 理解条件预测的动机和基本方法（条件 MLP、CVAE、条件扩散）。

---

## Part 4: 数据集与评估

### 主要数据集

| 数据集 | 规模 | 特点 |
|--------|------|------|
| **Argoverse 2 Motion Forecasting** | ~250K 场景 | Miami + Pittsburgh，路口丰富，社区活跃，最常用 |
| **Waymo Open Motion Dataset** | ~100K 场景 | 高质量，含交互标签，大型排行榜 |
| **nuScenes** | ~34K 场景 | 多模态传感器，适合做感知-预测联合 |

**学什么：**
- 三个数据集的格式差异（Argoverse 用 JSON + Parquet，Waymo 用 Protobuf）。
- 各自的数据加载 API。
- 场景长度、采样频率、标签种类等细节。

**学到什么程度：** 至少熟悉一个数据集的完整加载和可视化流程。推荐从 Argoverse 2 入手（社区工具最完善）。

**资源：**
- Argoverse 2：`https://argoverse.github.io/`
- Waymo Open：`https://waymo.com/open/`
- nuScenes：`https://www.nuscenes.org/`

### 评估指标

1. **minADE（minimum Average Displacement Error）**：K 条预测轨迹中，与真值最接近的那条的平均位移误差（逐点 L2 距离取平均）。衡量"最佳预测的精度"。

2. **minFDE（minimum Final Displacement Error）**：K 条预测轨迹中，终点与真值终点最接近的距离。衡量"终点的准确性"，对到达哪里更重要。

3. **Miss Rate（MR）**：K 条预测轨迹的终点距离真值终点超过 2 米的比例。衡量"是否预测到了正确的大方向"。

4. **MRPC（Miss Rate at Prediction Center）**：将 K 条轨迹聚类为中心预测后，评价该中心的 miss rate。衡量"多模态预测的覆盖性"。

**学什么：** 务必理解每个指标的数学定义和物理含义。例如 minADE 更关注平滑性，minFDE 更关注终点准确性，Miss Rate 更关注是否"击中"正确的大意图。

**学到什么程度：** 能自己实现这些评估指标的计算代码。建议从零手写一遍，而不是直接调包。

---

## Part 5: 就业与学习建议

### 预测方向的就业前景

运动预测是自动驾驶中比较核心的算法方向之一，就业面较广：

- **国内主要公司**：百度 Apollo、华为车 BU、小鹏、蔚来、理想、华为、地平线、Momenta、商汤等都有预测算法岗。
- **岗位特点**：预测算法岗竞争适中，相比感知岗人数略少，但岗位数量也不算多。薪资水平与感知、规划岗位基本持平。
- **趋势**：端到端自动驾驶（End-to-End AD）正在兴起，但短期内独立的预测模块仍然不可替代。长期来看，预测能力会融入到端到端框架中，但建模方法论不变。

### 适合作为研究生课题吗？

**适合**，原因如下：
1. 研究问题明确，有标准化的评测指标和排行榜。
2. 数据集公开且质量高，适合独立开展研究。
3. 与感知、规划都有交叉，研究空间大（感知-预测联合、预测-规划联合）。
4. 工业界需求稳定，读研期间做的工作容易转化为工业能力。

**注意事项：** 该领域已经相对成熟，单纯的模型架构改进越来越难发顶会。建议关注新方向：闭环评估、真实世界部署、多模态（视觉+轨迹）联合、生成式方法等。

### 需要的基础知识

| 基础领域 | 具体内容 | 优先级 |
|---------|---------|--------|
| **深度学习基础** | PyTorch、CNN、RNN/LSTM、注意力机制 | 必须 |
| **Transformer** | Self-Attention、Cross-Attention、Positional Encoding | 必须 |
| **图神经网络** | GNN、GCN、GAT 的基本概念 | 高 |
| **3D几何** | 坐标系变换、旋转矩阵、投影 | 高 |
| **概率统计** | 高斯分布、混合高斯、KL 散度、贝叶斯 | 中高 |
| **优化理论** | 损失函数设计、anchor matching | 中 |
| **扩散模型** | DDPM 基本原理（选学） | 中 |

### 推荐学习路径（约 6 个月）

**第 1-2 月：基础巩固**
- 学习 PyTorch 深度学习实战（推荐李沐《动手学深度学习》）
- 精读 Transformer 和 GNN 经典论文
- 熟悉 Argoverse 2 数据集，完成数据加载和可视化

**第 3-4 月：核心论文精读与复现**
- 按 VectorNet → HiVT → MTR → QCNet 的顺序精读
- 复现至少一篇方法（推荐从 HiVT 开始，代码质量高）
- 在 Argoverse 2 上跑通训练和评估，提交排行榜

**第 5-6 月：深入与创新**
- 阅读 2023-2025 年的最新论文（扩散模型、端到端、闭环评估等）
- 在 baseline 基础上尝试改进，形成自己的研究想法
- 关注顶级会议的最新预测方向论文（CVPR、NeurIPS、ICCV、CoRL）

**论文阅读清单（按顺序）：**
1. VectorNet (`arxiv:2005.04259`)
2. TNT (`arxiv:2008.08294`)
3. HiVT (`arxiv:2205.01170`)
4. MTR (`arxiv:2209.13508`)
5. QCNet (`arxiv:2306.10508`)
6. Scene Transformer (`arxiv:2206.08756`)
7. MotionDiffuser (`arxiv:2306.14840`)
8. UniTraj (`arxiv:2403.15098`)

**GitHub 仓库推荐：**
- `sshaoshuai/MTR`：MTR 官方实现，代码规范
- `ZikangZhou/QCNet`：QCNet 实现，Argoverse 2 SOTA
- `ZikangZhou/HiVT`：HiVT 实现，适合入门
- `uber-research/VectorNet`：VectorNet 实现
- `waymo-research/waymo-open-dataset`：Waymo 官方工具包

---

**总结：** 运动预测是一个理论扎实、工程落地性强的研究方向。核心在于理解场景编码、多模态预测和交互建模三个要素。建议从 VectorNet 起步，逐步深入到 MTR/QCNet 等前沿方法，同时在 Argoverse 2 上实践全流程。保持对最新研究的关注，找到有区分度的研究切入点。

---

# 模块十：技术社区与信息源指南

## 自动驾驶技术社区与信息源指南

---

### Part 1: 中文技术社区

**微信公众号（建议全部关注，日常碎片化阅读）**

- **自动驾驶之心**：国内最活跃的自动驾驶垂直公众号，覆盖感知、规划、定位全栈。日常推送论文解读、行业动态、招聘信息。建议每天花10分钟扫标题，周末精读感兴趣的论文解读。
- **3D视觉工坊**：专注点云处理、3D目标检测、SLAM方向。经常发布代码复现教程和数据集解读，适合入门阶段快速建立技术直觉。
- **泡泡机器人SLAM**：SLAM方向的中文权威社区，每周有论文速递和线上讲座回放。做定位方向的同学必须关注，有专门的SLAM学习路线图。
- **计算机视觉life**：CV基础扎实，经常拆解经典网络结构，适合打基础。配套B站视频和代码仓库，学习门槛低。
- **新智元 / 机器之心 / 量子位**：行业新闻类，了解商业化进展和公司动态，不用精读，知道发生了什么即可。

**知乎**

- 关注话题：「自动驾驶」「计算机视觉」「SLAM」「激光雷达」「BEV感知」
- 搜索技巧：在知乎搜索框输入关键词后选"专栏"筛选，能找到大量系列文章。例如搜索"自动驾驶规控"能找到多位从业者的技术复盘。
- 建议：看到高质量回答后点进作者主页，关注其专栏。很多从业者会在知乎连载学习笔记。

**B站UP主**

- **古月居**：ROS入门首选，从ROS1到ROS2的完整教程链，配套代码全部开源。建议研一上学期跟着做完全部基础教程。
- **鱼香ROS**：ROS2实战教程，项目驱动式教学。适合在古月居基础教程之后进阶学习。
- **3D视觉工坊**：与公众号同源，B站上有完整的点云处理和3D检测课程，部分免费。
- **深蓝学院**：自动驾驶全栈课程（感知、定位、规划、控制），付费课程质量较高，经常有优惠活动。
- **小林coding / 蓝桥云课**：面试算法题和编程基础补充。

**CSDN**

- 不建议长期依赖CSDN，但它是中文搜索代码报错最快的平台。遇到环境配置、库版本冲突问题，用「关键词 + CSDN」在百度搜索通常能快速解决。
- 关注几位高质量博主：搜索「自动驾驶」后按粉丝数排序，前几位博主的系列文章值得收藏。

---

### Part 2: 英文技术社区

**arXiv论文跟踪**

- 每日浏览：`arxiv.org/list/cs.CV/recent`（计算机视觉）、`cs.RO`（机器人）、`cs.AI`（人工智能）
- 高效工具：用 **arxiv-sanity**（stanford.edu/~karpathy/arxiv-sanity-lite/）做个性化推荐，标记感兴趣的论文后系统自动推送相似论文。
- 建议：每天花20分钟扫标题和Abstract，每周精读2-3篇。建一个Zotero文献库管理论文。

**Papers With Code**

- 网址：`paperswithcode.com`
- 用途：查找某个任务（如3D目标检测）的SOTA排行榜，直接找到对应论文和开源代码。写论文Related Work部分时必备。
- 技巧：进入某个Benchmark页面，按"Stars"排序可以快速找到社区认可度最高的方法。

**GitHub**

- Trending页面：`github.com/trending`，选择语言为Python/C++，每周看一次，发现新兴项目。
- 建立自己的Awesome列表：创建一个私有仓库，按方向（感知/规划/数据集）分类收藏优质项目。

**Reddit**

- **r/SelfDrivingCars**：行业动态和讨论，适合了解公众对自动驾驶的看法和商业进展。注意区分技术讨论和情绪化评论。
- **r/robotics**：机器人技术综合社区，偏硬件和控制方向。
- **r/computervision**：CV技术问答，遇到论文细节不理解时可以发帖提问。

**Twitter/X**

- 关注关键人物：**Andrej Karpathy**（前Tesla AI总监）、**Yann LeCun**（Meta首席AI科学家）、**Raquel Urtasun**（Waabi创始人）、**Drago Anguelov**（Waymo研究负责人）
- 关注机构账号：`@Waymo`、`@Cruise`、`@ApolloPlatform`、`@NVIDIAAIDev`
- 用法：不用每天刷，每周看一次"Highlights"即可，主要获取一手行业资讯和论文预告。

---

### Part 3: 重要会议与时间表

| 会议 | 方向 | 投稿截止（大约） | 录取率 | 投稿建议 |
|------|------|-----------------|--------|----------|
| **CVPR** | 视觉+自动驾驶感知 | 每年11月中旬 | ~25% | 感知、BEV、点云检测的首选。周期长，质量要求高，建议研二上学期开始准备 |
| **ICCV** | 视觉（双数年） | 每年3月 | ~26% | 与CVPR同级，双数年举办。论文风格偏方法创新 |
| **ECCV** | 视觉（双数年） | 每年3月 | ~28% | 与ICCV错开的双数年会议，略低于CVPR的难度 |
| **ICRA** | 机器人 | 每年9月 | ~40% | 规划、控制、机器人系统方向首选。偏工程实现和实验验证 |
| **IROS** | 机器人 | 每年3月 | ~45% | ICRA的姊妹会，录取率略高，适合投系统类和应用类工作 |
| **IV** | 智能车辆 | 每年2月 | ~50% | 自动驾驶领域专属会议，偏车辆工程和系统集成 |
| **ITSC** | 智能交通 | 每年3月 | ~55% | 交通场景、V2X、交通流仿真方向。难度相对低，适合首次投稿 |
| **CoRL** | 机器人学习 | 每年6月 | ~30% | 强化学习、模仿学习在机器人中的应用。偏学习方法 |
| **RSS** | 机器人 | 每年1月 | ~30% | 小而精的顶级机器人会议，理论深度要求高 |

**投稿策略建议**：研一积累，研二上投第一篇（建议从IV/ITSC或Workshop起步），研二下冲刺CVPR/ICRA。Workshop论文虽然权重低，但门槛低、周期短，适合练手和建立信心。

---

### Part 4: 学习平台

**系统课程**

- **Coursera - Self-Driving Cars Specialization（多伦多大学）**：4门课从感知到端到端，有编程作业。建议研一上学期完成，可申请助学金免费学习。
- **Udacity - Self-Driving Car Engineer Nanodegree**：实战项目驱动，包含车道线检测、路径规划等。费用较高，但项目可以直接放进简历。
- **深蓝学院 - 自动驾驶全栈课程**：中文授课，覆盖感知、定位、规划、控制。性价比高于Udacity，社区答疑活跃。
- **Stanford CS231n / CS224n**：深度学习和NLP基础课，B站有中文字幕版。做感知方向必须过一遍CS231n。

**B站免费资源**

- 古月居ROS教程（入门）
- 王道考研408系列（补计算机基础）
- 各大顶会的录播报告（搜索"CVPR 2024 oral"等）
- 李宏毅机器学习课程（理论基础）

**面试准备**

- **牛客网**：自动驾驶专项面经板块，搜索"自动驾驶"即可看到各公司的真题。建议研二下学期开始系统刷题。
- **LeetCode**：保持每周3-5题的手感，重点刷数组、树、图、动态规划。
- 手撕代码练习：在白板/纸上练习手写卷积、NMS、IoU计算等自动驾驶常见代码题。

---

### Part 5: 开源社区

**核心社区与参与方式**

- **ROS/ROS2**：加入ROS Discourse论坛（`discourse.ros.org`）和ROS Answers提问平台。建议从回答初学者问题开始建立社区影响力。
- **Autoware**：`autoware.org`，基于ROS2的全栈自动驾驶开源方案。加入其GitHub Organization可以参与开发讨论。适合做规控方向的同学深入研究其Planning模块。
- **Apollo**：百度开源自动驾驶平台，中文文档完善。加入Apollo开发者社区微信群（在Apollo官网申请），有问题直接在群里问。

**如何提PR**

1. 先Fork项目到自己的GitHub
2. 仔细阅读项目的CONTRIBUTING.md
3. 从标记为"good first issue"的Issue入手
4. 修完后创建PR，描述清楚改了什么、为什么改
5. 等待Review，根据反馈修改
- 第一个PR建议从修复文档错误或添加单元测试开始，门槛低且容易被接受。

**推荐Star列表**

- `open-mmlab/mmdetection3d`：3D目标检测工具箱
- `hustvl/BEVFormer`：BEV感知经典实现
- `PRBonn/kiss-icp`：简洁优雅的激光雷达里程计
- `commaai/openpilot`：量产级开源自动驾驶系统
- `zaiorn/awesome-autonomous-driving`：自动驾驶资源汇总

---

### Part 6: 求职信息

**面经与准备**

- **牛客网面经**：搜索"自动驾驶 感知"或"规控算法"等关键词，按公司筛选。重点看百度、华为、小鹏、蔚来的面经帖。
- **一亩三分地**：如果考虑海外公司（Waymo、Zoox、Nuro），这个论坛的面经非常全面。

**招聘时间线**

- **秋招（主战场）**：每年7-8月提前批，9-10月正式批。提前批竞争小，一定要参加。
- **春招（补招）**：次年2-4月，岗位数量少于秋招，但竞争也相对小。
- **实习**：大厂通常3-4月开放暑期实习申请，中小公司全年滚动招聘。

**实习信息获取**

- 牛客网实习板块
- 各公司官网直接投递
- 导师和学长内推（最高效的方式，主动询问）
- 微信公众号推送（自动驾驶之心等）
- LinkedIn领英（外企和合资公司的首选渠道）

**公司梯队参考**

- 第一梯队：Waymo、百度Apollo、华为车BU、小鹏、蔚来、Momenta
- 第二梯队：地平线、大疆车载、AutoX、图森未来、元戎启行
- 第三梯队：众多L2+方案公司和Tier1供应商（德赛西威、经纬恒润等）

**最后建议**：在研一期间，每类资源花半天时间浏览熟悉，然后根据自己的研究方向选择2-3个核心信息源坚持跟踪。信息过载比信息匮乏更危险，精读远比泛读重要。
