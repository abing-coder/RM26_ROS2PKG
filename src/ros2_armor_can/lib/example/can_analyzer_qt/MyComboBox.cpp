#include "mycombobox.h"
MyComboBox::MyComboBox(QWidget *parent) : QComboBox(parent)
{

}

void MyComboBox::appendItem(const QString &text, bool bChecked)
{
    QIcon icon;
    if (bChecked)
    {
        icon.addFile(":/check.png");
    }
    else
    {
        icon.addFile(":/uncheck.png");
    }
    addItem(icon, text, QVariant());
    checked.append(bChecked);
}


void MyComboBox::hidePopup()
{
    int iconWidth = this->iconSize().width();

    int x = QCursor::pos().x() - mapToGlobal(geometry().topLeft()).x() + geometry().x();

    int index = view()->selectionModel()->currentIndex().row();

    if (x >= 0 && x <= iconWidth)
    {
        updateIndexStatus(index);
    }
    else
    {
        QComboBox::hidePopup();
    }
}

void MyComboBox::mousePressEvent(QMouseEvent *event)
{
    int x = event->pos().x();

    int iconWidth = this->iconSize().width();

    if (x <= iconWidth)
    {
        int index = this->currentIndex();
        updateIndexStatus(index);
    }
    else
    {
        QComboBox::mousePressEvent(event);
    }
}

void MyComboBox::updateIndexStatus(int index)
{
    bool bChecked = this->checked[index];

    if (bChecked)
    {
        this->setItemIcon(index, QIcon(":/uncheck.png"));
    }
    else
    {
        this->setItemIcon(index, QIcon(":/check.png"));
    }

    this->checked[index] = !bChecked;
}

bool MyComboBox::isChecked(int row)
{
    bool bChecked = this->checked[row];
    return bChecked;
}




