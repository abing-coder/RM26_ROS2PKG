#ifndef MYCOMBOBOX_H
#define MYCOMBOBOX_H
#include <QWidget>
#include <QComboBox>
#include <QMouseEvent>
#include <QAbstractItemView>
#include <QItemSelectionModel>
#include <QList>
#include <QDebug>
#include "canmsgtablemodel.h"
//HULANG-BTB
//https://www.cnblogs.com/HULANG-BTB/p/8612654.html
class MyComboBox : public QComboBox
{
    Q_OBJECT
public:
    explicit MyComboBox(QWidget *parent = nullptr);
    //添加下拉框内容
    void appendItem(const QString &text, bool bChecked);

    //QComboBox的虚函数用来隐藏列表框,当单击是复选框时不让隐藏，用来改变状态
    void hidePopup();

    QMouseEvent *event;
    //获取下拉框状态
    bool isChecked(int row);

protected:
    void mousePressEvent(QMouseEvent *event);

private:
    //更新更改项的状态
    void updateIndexStatus(int index);

private:
    QList<int> checked;
};
#endif // MYCOMBOBOX_H
