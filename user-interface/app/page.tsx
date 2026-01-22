import { BatteryStatus, Header, EmergencyStop } from "./components";
import Dropdown from "./components/Dropdown";

export default function Home() {
  return (
    <div>
      <Header />
      <main className="bg-blue-950 h-screen p-8">
        <BatteryStatus />
        <EmergencyStop />
      </main>
    </div>
  );
}
