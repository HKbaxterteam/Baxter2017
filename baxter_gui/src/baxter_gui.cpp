#include "baxter_gui.h"
#include "ui_baxter_gui.h"

Baxter_gui::Baxter_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Baxter_gui)
{
    ui->setupUi(this);
}

Baxter_gui::~Baxter_gui()
{
    delete ui;
}

void Baxter_gui::on_pushButton_clicked()
{
    ui->label->setText("BaxterBaxterBaxterBaxterBaxterBaxter");
}

void Baxter_gui::on_pushButton_2_clicked()
{
    ui->label->setText(" ");
}
