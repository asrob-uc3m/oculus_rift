/********************************************************************************
** Form generated from reading UI file 'servo_oculus.ui'
**
** Created: Sun Sep 7 14:35:06 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SERVO_OCULUS_H
#define UI_SERVO_OCULUS_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Servo_Oculus
{
public:
    QVBoxLayout *verticalLayout_5;
    QVBoxLayout *verticalLayout_4;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_3;
    QSpacerItem *horizontalSpacer;
    QFrame *line;
    QHBoxLayout *horizontalLayout_5;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *ResetButton;
    QCheckBox *CheckManual;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QLabel *label;
    QLabel *label_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QSlider *PitchSlider;
    QSpinBox *PitchSpin;
    QLabel *label_4;
    QHBoxLayout *horizontalLayout_2;
    QSlider *YawSlider;
    QSpinBox *YawSpin;
    QLabel *label_5;

    void setupUi(QWidget *Servo_Oculus)
    {
        if (Servo_Oculus->objectName().isEmpty())
            Servo_Oculus->setObjectName(QString::fromUtf8("Servo_Oculus"));
        Servo_Oculus->resize(410, 164);
        Servo_Oculus->setMinimumSize(QSize(0, 164));
        Servo_Oculus->setMaximumSize(QSize(16777215, 164));
        verticalLayout_5 = new QVBoxLayout(Servo_Oculus);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_3 = new QLabel(Servo_Oculus);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        QFont font;
        font.setPointSize(15);
        font.setBold(false);
        font.setWeight(50);
        label_3->setFont(font);

        horizontalLayout_4->addWidget(label_3);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer);


        verticalLayout_3->addLayout(horizontalLayout_4);

        line = new QFrame(Servo_Oculus);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_3->addWidget(line);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_2);

        ResetButton = new QPushButton(Servo_Oculus);
        ResetButton->setObjectName(QString::fromUtf8("ResetButton"));

        horizontalLayout_5->addWidget(ResetButton);

        CheckManual = new QCheckBox(Servo_Oculus);
        CheckManual->setObjectName(QString::fromUtf8("CheckManual"));
        CheckManual->setChecked(true);

        horizontalLayout_5->addWidget(CheckManual);


        verticalLayout_3->addLayout(horizontalLayout_5);


        verticalLayout_4->addLayout(verticalLayout_3);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label = new QLabel(Servo_Oculus);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_2->addWidget(label);

        label_2 = new QLabel(Servo_Oculus);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_2->addWidget(label_2);


        horizontalLayout_3->addLayout(verticalLayout_2);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        PitchSlider = new QSlider(Servo_Oculus);
        PitchSlider->setObjectName(QString::fromUtf8("PitchSlider"));
        PitchSlider->setEnabled(false);
        PitchSlider->setMinimum(-33);
        PitchSlider->setMaximum(33);
        PitchSlider->setOrientation(Qt::Horizontal);

        horizontalLayout->addWidget(PitchSlider);

        PitchSpin = new QSpinBox(Servo_Oculus);
        PitchSpin->setObjectName(QString::fromUtf8("PitchSpin"));
        PitchSpin->setEnabled(false);
        PitchSpin->setMinimum(-45);
        PitchSpin->setMaximum(45);

        horizontalLayout->addWidget(PitchSpin);

        label_4 = new QLabel(Servo_Oculus);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout->addWidget(label_4);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        YawSlider = new QSlider(Servo_Oculus);
        YawSlider->setObjectName(QString::fromUtf8("YawSlider"));
        YawSlider->setEnabled(false);
        YawSlider->setMinimum(-45);
        YawSlider->setMaximum(45);
        YawSlider->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(YawSlider);

        YawSpin = new QSpinBox(Servo_Oculus);
        YawSpin->setObjectName(QString::fromUtf8("YawSpin"));
        YawSpin->setEnabled(false);
        YawSpin->setMinimum(-45);
        YawSpin->setMaximum(45);

        horizontalLayout_2->addWidget(YawSpin);

        label_5 = new QLabel(Servo_Oculus);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_2->addWidget(label_5);


        verticalLayout->addLayout(horizontalLayout_2);


        horizontalLayout_3->addLayout(verticalLayout);


        verticalLayout_4->addLayout(horizontalLayout_3);


        verticalLayout_5->addLayout(verticalLayout_4);


        retranslateUi(Servo_Oculus);

        QMetaObject::connectSlotsByName(Servo_Oculus);
    } // setupUi

    void retranslateUi(QWidget *Servo_Oculus)
    {
        Servo_Oculus->setWindowTitle(QApplication::translate("Servo_Oculus", "Servo_Oculus", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("Servo_Oculus", "Servo Control", 0, QApplication::UnicodeUTF8));
        ResetButton->setText(QApplication::translate("Servo_Oculus", "Reset", 0, QApplication::UnicodeUTF8));
        CheckManual->setText(QApplication::translate("Servo_Oculus", " Manual", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("Servo_Oculus", "Pitch", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("Servo_Oculus", "Yaw", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("Servo_Oculus", "Degrees", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("Servo_Oculus", "Degrees", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Servo_Oculus: public Ui_Servo_Oculus {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SERVO_OCULUS_H
