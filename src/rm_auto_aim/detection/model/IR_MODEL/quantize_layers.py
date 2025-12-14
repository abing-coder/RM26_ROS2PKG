#!/usr/bin/env python3
"""
量化IR模型的前四层卷积层权重为int8
使用OpenVINO IR模型格式
"""
import xml.etree.ElementTree as ET
import struct
import shutil

def fp16_to_fp32(fp16_bytes):
    """将fp16字节转换为fp32列表"""
    fp32_values = []
    for i in range(0, len(fp16_bytes), 2):
        if i + 1 < len(fp16_bytes):
            # 读取fp16值
            fp16_uint16 = struct.unpack('<H', fp16_bytes[i:i+2])[0]

            # 提取符号位、指数和尾数
            sign = (fp16_uint16 >> 15) & 0x1
            exponent = (fp16_uint16 >> 10) & 0x1F
            mantissa = fp16_uint16 & 0x3FF

            # 转换为fp32
            if exponent == 0:
                if mantissa == 0:
                    fp32 = -0.0 if sign else 0.0
                else:
                    # 非规格化数
                    fp32 = ((-1) ** sign) * (mantissa / 1024.0) * (2 ** -14)
            elif exponent == 31:
                if mantissa == 0:
                    # 无穷大
                    fp32 = float('-inf') if sign else float('inf')
                else:
                    # NaN
                    fp32 = float('nan')
            else:
                # 规格化数
                fp32 = ((-1) ** sign) * (1.0 + mantissa / 1024.0) * (2 ** (exponent - 15))

            fp32_values.append(fp32)

    return fp32_values

def quantize_fp32_to_int8(fp32_list):
    """将fp32列表量化为int8"""
    # 计算最大绝对值用于对称量化
    max_abs = max(max(abs(v) for v in fp32_list), 1e-6)

    # 量化到int8范围[-127, 127]
    int8_list = []
    for v in fp32_list:
        quantized = round(v / max_abs * 127)
        quantized = max(-128, min(127, quantized))
        int8_list.append(quantized)

    return int8_list, max_abs

def main():
    xml_path = 'new.xml'
    bin_path = 'new.bin'
    output_xml_path = 'new_int8.xml'
    output_bin_path = 'new_int8.bin'

    print("=" * 60)
    print("IR模型前四层卷积层int8量化工具")
    print("=" * 60)
    print(f"输入: {xml_path}, {bin_path}")
    print(f"输出: {output_xml_path}, {output_bin_path}\n")

    # 解析XML文件
    tree = ET.parse(xml_path)
    root = tree.getroot()
    layers = root.findall('.//layer')

    # 找到所有卷积层
    conv_layers = []
    for layer in layers:
        if layer.get('type') == 'Convolution':
            conv_layers.append(layer)

    num_conv = len(conv_layers)
    num_to_quantize = min(4, num_conv)

    print(f"发现 {num_conv} 个卷积层，将量化前 {num_to_quantize} 层\n")

    # 复制文件到输出
    shutil.copy2(xml_path, output_xml_path)
    shutil.copy2(bin_path, output_bin_path)

    # 重新解析以便修改
    tree_out = ET.parse(output_xml_path)
    root_out = tree_out.getroot()
    layers_out = root_out.findall('.//layer')

    results = []

    for i in range(num_to_quantize):
        conv_layer = conv_layers[i]
        layer_id = int(conv_layer.get('id'))
        layer_name = conv_layer.get('name')

        print(f"第 {i+1} 层: {layer_name} (ID: {layer_id})")

        # 查找对应的权重Const层（通常是id-2，因为中间有Convert层）
        const_layer = None
        for layer in layers:
            layer_id_str = layer.get('id')
            if layer_id_str:
                try:
                    id_num = int(layer_id_str)
                    # 权重层通常是 conv_layer_id - 2
                    if id_num == layer_id - 2 and layer.get('type') == 'Const':
                        const_layer = layer
                        break
                except ValueError:
                    continue

        if const_layer is None:
            print(f"  ⚠️  未找到权重层，跳过此层\n")
            continue

        const_id = const_layer.get('id')
        const_name = const_layer.get('name')

        # 获取权重数据信息
        data_elem = const_layer.find('data')
        if data_elem is None:
            print(f"  ⚠️  未找到数据元素，跳过此层\n")
            continue

        offset = int(data_elem.get('offset'))
        size = int(data_elem.get('size'))
        shape = data_elem.get('shape')
        element_type = data_elem.get('element_type')

        print(f"  权重层: {const_name} (ID: {const_id})")
        print(f"  形状: {shape}")
        print(f"  原始精度: {element_type}")
        print(f"  位置: offset={offset}, size={size} bytes")

        # 读取fp16权重
        with open(bin_path, 'rb') as f:
            f.seek(offset)
            fp16_bytes = f.read(size)

        if len(fp16_bytes) != size:
            print(f"  ⚠️  读取字节数不匹配: {len(fp16_bytes)} vs {size}\n")
            continue

        # 转换为fp32
        fp32_weights = fp16_to_fp32(fp16_bytes)
        print(f"  权重数量: {len(fp32_weights)}")

        # 量化为int8
        int8_weights, max_abs = quantize_fp32_to_int8(fp32_weights)
        # 转换为字节（uint8范围0-255）
        int8_bytes = bytes((w + 128) % 256 for w in int8_weights)

        print(f"  量化范围: ±{max_abs:.6f}")
        print(f"  量化后大小: {len(int8_bytes)} bytes")

        # 写入新的bin文件
        with open(bin_path, 'rb') as f:
            bin_content = f.read()

        with open(output_bin_path, 'wb') as f:
            f.write(bin_content[:offset])
            f.write(int8_bytes)
            f.write(bin_content[offset + len(int8_bytes):])

        # 更新输出XML中的Const层
        for layer in layers_out:
            if layer.get('id') == const_id:
                data_elem_out = layer.find('data')
                if data_elem_out is not None:
                    data_elem_out.set('element_type', 'i8')
                    data_elem_out.set('size', str(len(int8_bytes)))
                    # 添加量化参数以便反量化
                    data_elem_out.set('quantization_scale', str(max_abs))
                break

        compression_ratio = size / len(int8_bytes)
        results.append({
            'layer': layer_name,
            'shape': shape,
            'original_size': size,
            'new_size': len(int8_bytes),
            'compression_ratio': compression_ratio
        })

        print(f"  ✓ 量化完成，压缩比: {compression_ratio:.2f}x\n")

    # 保存修改后的XML
    tree_out.write(output_xml_path, encoding='utf-8', xml_declaration=True)

    # 输出结果汇总
    print("=" * 60)
    print("量化完成!")
    print("=" * 60)

    if results:
        total_original = sum(r['original_size'] for r in results)
        total_new = sum(r['new_size'] for r in results)

        print(f"\n成功量化 {len(results)} 个卷积层:")
        print(f"\n{'层名':<30} {'形状':<20} {'原始大小':>12} {'量化后':>12} {'压缩比':>8}")
        print("-" * 90)

        for r in results:
            print(f"{r['layer']:<30} {r['shape']:<20} "
                  f"{r['original_size']:>8} bytes {r['new_size']:>10} bytes "
                  f"{r['compression_ratio']:>6.2f}x")

        print("-" * 90)
        print(f"{'总计':<30} {'':<20} "
              f"{total_original:>8} bytes {total_new:>10} bytes "
              f"{total_original/total_new:>6.2f}x")

        print(f"\n输出文件:")
        print(f"  {output_xml_path}")
        print(f"  {output_bin_path}")

        print(f"\n注意: 此脚本使用简单的对称量化方案。")
        print(f"如需更高精度，建议使用OpenVINO POT工具进行校准量化。")
    else:
        print("⚠️  没有成功量化任何层")

if __name__ == '__main__':
    main()
