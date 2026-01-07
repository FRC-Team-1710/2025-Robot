package frc.robot.utils.customLogger;

import edu.wpi.first.math.Pair;
import frc.robot.Robot;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class SimplyLogger {
  private final SimplyConfig config;

  private final Robot robot;

  private Map<Pair<SimplyLogged, String>, String> relationMap = new HashMap<>();

  private Set<Class<?>> visited = new HashSet<>();

  public SimplyLogger(Robot robot) {
    this(new SimplyConfig(), robot);
  }

  public SimplyLogger(SimplyConfig config, Robot robot) {
    this.config = config;
    this.robot = robot;
  }

  public void periodic() throws NoSuchFieldException, SecurityException {
    for (Pair<SimplyLogged, String> key : relationMap.keySet()) {
      var temp = key.getFirst().getClass().getDeclaredField(key.getSecond());
      // config.backend.log(relationMap.get(key), temp.);
    }
  }

  public void scan(Class<?> rootClass, Class<?> currentClass, String currentPath) throws Exception {
    if (currentClass == null || isSystemClass(currentClass) || visited.contains(currentClass)) {
      return;
    }

    visited.add(currentClass);

    for (Field field : currentClass.getDeclaredFields()) {
      String newPath = "";

      if (field.isAnnotationPresent(SimplyLogged.class)) {
        SimplyLogged ann = field.getAnnotation(SimplyLogged.class);
        String annotationValue = ann.name();
        newPath = currentPath.isEmpty() ? annotationValue : currentPath + "/" + annotationValue;

        String entry = rootClass.getSimpleName() + "/" + newPath;
        if (!relationMap.containsKey(new Pair<SimplyLogged, String>(ann, currentClass.getName()))) {
          relationMap.put(new Pair<SimplyLogged, String>(ann, currentClass.getName()), entry);
        } else {
          throw new Exception(
              "You did something wrong! Looks like there's 2 variables being logged as "
                  + ann.name()
                  + " at "
                  + entry
                  + " & "
                  + relationMap.get(new Pair<SimplyLogged, String>(ann, currentClass.getName())));
        }
      }

      scan(rootClass, field.getType(), newPath);
    }
  }

  private boolean isSystemClass(Class<?> clazz) {
    String name = clazz.getName();
    return name.startsWith("java.")
        || name.startsWith("javax.")
        || name.startsWith("sun.")
        || name.startsWith("jdk.");
  }

  @SuppressWarnings("unused")
  public Map<Pair<SimplyLogged, String>, String> getResults() {
    if (relationMap.isEmpty()) {
      try {
        scan(robot.getClass(), robot.getClass(), config.root);
      } catch (Exception e) {
        System.out.println(e);
        if (config.crashOnError) {
          var killer = 1 / 0;
        }
      }
    }
    return relationMap;
  }
}
