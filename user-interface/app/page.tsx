import { BatteryStatus, Header } from "./components";
import DummyWebsocketDisplay from "./components/DummyWebsocketDisplay";

export default function Home() {
  return (
    <div>
      <Header />
      <main className="bg-blue-950 h-screen p-8">
        <BatteryStatus />
        <DummyWebsocketDisplay/>
      </main>
    </div>
  );
}
