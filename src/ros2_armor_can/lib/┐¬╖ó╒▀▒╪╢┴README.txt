二次开发包结构：
/doc：API明细手册，需要二次开发的客户请务必阅读BMAPI.chm
/bin：本二次开发库的可执行文件和动态库文件，同时附送Windows下可立即执行的上位机can_analyzer_qt以及can_analyzer_csharp等，您可以直接双击运行
/lib：本二次开发库的静态库文件，仅用于Windows下开发，方便链接器使用DLL中的导出符号
/include：本二次开发库的头文件
/ref：一些第三方参考资料，供您查阅参考
/example/bmapi_receive_only: 最基础的C/C++命令行例程，演示如何进行报文接收
/example/bmapi_transmit_only: 最基础的C/C++命令行例程，演示如何进行报文发送
/example/bmapi_test：简单的C/C++命令行例程，用于演示各API的基础用法，也可以用来做收发测试（需要使用不同的命令行参数启动两个实例来执行对发）
/example/multichannel_rx_cpp：简单的C/C++命令行例程，演示如何进行高效率多通道并行接收
/example/multichannel_isotp_txrx_cpp: 简单的C/C++命令行例程，基于bmapi_test，但是额外演示了如何同时通过多个通道来进行ISOTP（ISO15765）通信，例如通过UDS36服务来实现多通道并行刷写（需要使用不同的命令行参数启动两个实例来执行对发）
/example/bmapi_cyclic_tx_task：演示如何使用BUSMUST特有的TXTASK API来进行周期性报文发送（例如网络管理报文）
/example/can_analyzer_csharp：C#窗口例程，实现了完整的上位机收发功能
/example/can_analyzer_qt：Qt窗口例程，实现了完整的上位机收发功能
/example/can_analyzer_pyqt：pyqt窗口例程，实现了完整的上位机收发功能，此工程可以直接在Ubuntu下作为调试专用上位机使用，直接运行/example/can_analyzer_pyqt/run.sh启动窗口即可
/example/can_analyzer_labview：Labview例程，实现了完整的上位机收发功能
/example/can_analyzer_vb.net：VB.NET例程，实现了完整的上位机收发功能
/example/python-can-3.3.4：Python-can扩展包V3，使用标准的python-can接口，支持BUSMUST全部系列产品，可在Windows和Ubuntu下使用
/example/python-can-4.0.0：Python-can扩展包V4，使用标准的python-can接口，支持BUSMUST全部系列产品，可在Windows和Ubuntu下使用
/example/python-udsoncan-1.13.1：udsoncan扩展包，支持UDS（ISO14229）全部服务，可在Windows和Ubuntu下使用
/example/uds_security_dll：用于在Busmaster中加载的UDS27解锁算法工程模板，您可以使用C语言自行编写解锁算法，使用此工程模板编译生成DLL，并在Busmaster中加载用于自动UDS测试

！！！Python开发者请注意！！！
由于Python的依赖复杂，请在开始开发之前务必补充阅读：Python开发必读.txt

以上，需要技术支持请前往微信搜索“霸码科技”，关注公众号后点击“技术支持”即可与技术人员一对一聊天获得深度技术支持。

感谢您的支持！
