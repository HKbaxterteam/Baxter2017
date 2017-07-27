#ifndef BAXTER_GUI_H
#define BAXTER_GUI_H

#include <QMainWindow>

namespace Ui {
class Baxter_gui;
}

class Baxter_gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit Baxter_gui(QWidget *parent = 0);
    ~Baxter_gui();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::Baxter_gui *ui;
};

#endif // BAXTER_GUI_H
