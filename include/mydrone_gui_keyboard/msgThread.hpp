#include <QThread>
#include <mydrone_gui_keyboard/qnode.hpp>

namespace mydrone_gui_keyboard {

class MsgThread : public QThread
{
  Q_OBJECT //왜 넣는지 나중에 알아보기
public:
  MsgThread(QNode* q);
  virtual ~MsgThread();
  void run();

Q_SIGNALS:
  void finish();
private:
  QNode* qnode;


};

}
